/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2023 Marvell.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <misc/mrvl_cn10k_dpi.h>

#include <rte_common.h>
#include <rte_malloc.h>
#include <rte_lcore.h>
#include <rte_dmadev.h>

#define DPI_NB_DESCS	1024
#define DPI_MAX_MOLR	1024
#define DPI_MAX_MPS	1024
#define DPI_MIN_MPS	128
#define DPI_MAX_MRRS	1024
#define DPI_MIN_MRRS	128
#define DPI_BURST_REQ	256
#define MAX_POINTERS	15
#define MAX_DPI_VFS	32
#define MAX_DPI_ENGS	6

static volatile bool force_quit;

struct dpi_test_dev_s {
	int dev_id;
	struct rte_dma_info dev_info;
	struct rte_dma_vchan_conf vchan_conf;
};

struct dpi_test_dev_s dpi_test[MAX_DPI_VFS];
rte_iova_t raddr = 0, laddr;
int mode, n_iter = 1;
int ptrs_per_instr = 1;
int burst_size = 64;
uint16_t done_count = 8;
uint16_t nb_dma_devs;
uint16_t pem_id;
uint64_t data_size = 128;
uint16_t mps = 128;
uint16_t mrrs = 256;
uint64_t fifo_mask = 0x10101010; 

uint64_t dma_submit_cnt[MAX_DPI_VFS] = { 0 };
uint64_t last_dma_submit_cnt[MAX_DPI_VFS] = { 0 };
uint64_t total_dma_cnt;
static uint64_t timer_period = 1; /* default period is 1 seconds */

char dpi_node[] = "/dev/mrvl-cn10k-dpi";

static void
signal_handler(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		printf("\n\nSignal %d received, preparing to exit...\n", signum);
		force_quit = true;
	}
}

static inline void dump_stats(void)
{
	int i;
	uint64_t tot = 0;

	for (i = 0; i < nb_dma_devs; i++) {
		printf("DMA %d Count %ld %s %2.2f Gbps\n", i,
			(dma_submit_cnt[i] - last_dma_submit_cnt[i]),
			(mode == 3 ? (i % 2 ? "Outb":"Inb"):""),
			((dma_submit_cnt[i] - last_dma_submit_cnt[i]) *
			 data_size * ptrs_per_instr * 8) / 1000000000.0);
		tot += dma_submit_cnt[i];
		last_dma_submit_cnt[i] = dma_submit_cnt[i];
	}
	printf("\ntot %ld tot_dma %ld Total: %ld Perf:%2.2f Gbps\n", tot,
	       total_dma_cnt, (tot - total_dma_cnt),
	       ((tot - total_dma_cnt) * data_size * ptrs_per_instr * 8)/1000000000.0);

	total_dma_cnt = tot;
}

static int
dma_perf_test(__attribute__((unused)) void *ptr)
{
	int ret = 0, vchan = 0, i, j;
	struct rte_dma_vchan_conf *vconf;
	unsigned int dma_port = rte_lcore_id() - 1;
	uint8_t num_req, max_ptr = ptrs_per_instr;
	uint8_t *remote_addr = (uint8_t *)raddr + (dma_port * 0x4000000ull);
	uint8_t *fptr[DPI_BURST_REQ][MAX_POINTERS];
	uint8_t *lptr[DPI_BURST_REQ][MAX_POINTERS];
	rte_iova_t src_ptr;
	rte_iova_t dst_ptr;
	struct rte_dma_sge src[DPI_BURST_REQ][MAX_POINTERS], dst[DPI_BURST_REQ][MAX_POINTERS];
	int xfer_mode, direction;
	uint64_t prev_tsc = 0, diff_tsc = 0, cur_tsc = 0;
	uint16_t nb_done = 0, last_idx = 0;
	bool dma_error = false;

	/* If lcore >= nb_dma_devs skip processing */
	if (dma_port >= nb_dma_devs)
		return 0;

	/* for dual mode use even dma devs for inbound DMA
	 * and odd dma devs for outbound DMA
	 */
	if (mode == 3)
		xfer_mode = (dma_port % 2) ? 2 : 1;
	else
		xfer_mode = mode;

	if (xfer_mode == 1)
		direction = RTE_DMA_DIR_DEV_TO_MEM;
	else if (xfer_mode == 2)
		direction = RTE_DMA_DIR_MEM_TO_DEV;
	else
		direction = RTE_DMA_DIR_MEM_TO_MEM;

	printf("dma_port %d, raddr %p mode %d, xmode %d dir %d\n", dma_port, remote_addr,
		mode, xfer_mode, direction);

	/* Alloc ptrs */
	for (i = 0; i < burst_size; i++) {
		for (j = 0; j < max_ptr; j++) {
			lptr[i][j] = remote_addr;
			fptr[i][j] = (uint8_t *)rte_malloc("xfer_block", data_size, 128);
			if (!fptr[i][j]) {
				printf("Unable to allocate internal memory\n");
				return -ENOMEM;
			}
			src_ptr = rte_malloc_virt2iova(fptr[i][j]);

			/* alloc for internal-only DMA */
			if (!xfer_mode) {
				lptr[i][j] = (uint8_t *)rte_malloc("xfer_block", data_size, 128);
				if (!lptr[i][j]) {
					printf("Unable to allocate internal memory\n");
					return -ENOMEM;
				}
				dst_ptr = rte_malloc_virt2iova(lptr[i][j]);
			} else {
				lptr[i][j] = lptr[i][j] + (j  * 64 * 1024);
				dst_ptr = src_ptr;
			}

			switch (direction) {
			/* outbound */
			case RTE_DMA_DIR_MEM_TO_DEV:
				src[i][j].addr = src_ptr;
				src[i][j].length = data_size;
				dst[i][j].addr = (rte_iova_t)lptr[i][j];
				dst[i][j].length = data_size;
				break;
			/* inbound */
			case RTE_DMA_DIR_DEV_TO_MEM:
				src[i][j].addr = (rte_iova_t)lptr[i][j];
				src[i][j].length = data_size;
				dst[i][j].addr = dst_ptr;
				dst[i][j].length = data_size;
				break;
			/* internal_only */
			case RTE_DMA_DIR_MEM_TO_MEM:
				src[i][j].addr = src_ptr;
				src[i][j].length = data_size;

				dst[i][j].addr = dst_ptr;
				dst[i][j].length = data_size;
				break;
			};
		}
	}

	vconf = &dpi_test[dma_port].vchan_conf;
	vconf->direction = direction;
	vconf->nb_desc = DPI_NB_DESCS;

	switch (direction) {
	/* outbound */
	case RTE_DMA_DIR_MEM_TO_DEV:
		vconf->dst_port.port_type = RTE_DMA_PORT_PCIE;
		vconf->dst_port.pcie.coreid = pem_id;
		break;
	/* inbound */
	case RTE_DMA_DIR_DEV_TO_MEM:
		vconf->src_port.port_type = RTE_DMA_PORT_PCIE;
		vconf->src_port.pcie.coreid = pem_id;
		break;
	/* internal_only */
	case RTE_DMA_DIR_MEM_TO_MEM:
		break;
	};

	vchan = rte_dma_vchan_setup(dma_port, vchan, vconf);
	if (vchan < 0) {
		ret = vchan;
		printf("DMA vchan setup failed with err %d\n", ret);
		goto free_buf;
	}

	rte_dma_start(dma_port);
	for (i = 0; i < burst_size; i++) {
		ret = rte_dma_copy_sg(dma_port, vchan, src[i], dst[i], max_ptr,
					 max_ptr, RTE_DMA_OP_FLAG_SUBMIT);
		if (ret < 0) {
			printf("dmadev copy op failed, ret=%d\n", ret);
			goto free_buf;
		}
	}
	num_req = i;

	do {
		if (unlikely(dma_port == rte_get_main_lcore())) {
			cur_tsc = rte_rdtsc();
			diff_tsc = cur_tsc - prev_tsc;
			if ((timer_period > 0) && (diff_tsc > timer_period)) {
				dump_stats();
				prev_tsc = cur_tsc;
			}
		}

		if (unlikely(force_quit)) {
			printf("dma_port %d quitting.\n", dma_port);
			sleep(3);
			goto free_buf;
		}

		nb_done = 0;
		do {
			nb_done += rte_dma_completed(dma_port, vchan, num_req, &last_idx,
						     &dma_error);
		} while (nb_done < done_count);

		dma_submit_cnt[dma_port] += nb_done;
		num_req = nb_done;

		for (i = 0; i < num_req; i++) {
			ret = rte_dma_copy_sg(dma_port, vchan, src[i], dst[i], max_ptr,
						 max_ptr, 0);
			if (ret < 0) {
				printf("dmadev copy_sg op failed, ret=%d\n", ret);
				goto free_buf;
			}
		}

		rte_dma_submit(dma_port, vchan);
	} while (1);

free_buf:
	printf("dma_port %d freeing memory.\n", dma_port);
	for (i = 0; i < burst_size; i++) {
		for (j = 0; j < max_ptr; j++) {
			if (fptr[i][j])
				rte_free(fptr[i][j]);
			if (!xfer_mode && lptr[i][j])
				rte_free(lptr[i][j]);
		}
	}

	return 0;
}

static uint64_t dpi_parse_addr(const char *q_arg)
{
	char *end = NULL;
	uint64_t n;

	/* parse number string */
	n = strtoul(q_arg, &end, 0);
	if ((q_arg[0] == '\0') || (end == NULL) || (*end != '\0'))
		return 0;

	return n;
}

/* display usage */
static void dpi_usage(const char *prgname)
{
	printf("%s [EAL options] --\n"
		"  -r <remote address>: Remote pointer\n"
		"  -l <first address>: This is also remote address valid for external only mode\n"
		"  -m <mode>: Mode of transfer\n"
		"             0: Internal Only\n"
		"             1: Inbound\n"
		"             2: Outbound\n"
		"             3: Dual - both inbound and outbound (supported only in perf test)\n"
		"             4: Inbound Latency\n"
		"             5: Outbound Latency\n"
		"             6: Inbound roundtrip Latency\n"
		"             7: Outbound roundtrip Latency\n"
		"             8: All modes 4, 5, 6, 7\n"
		"		Max data size for latency tests :\n"
		"			(65535*15) = 983025 bytes\n"
		"  -i <iteration>: No.of iterations\n"
		"  -s <data size>: Size of data to be DMA'ed (Default is 256)\n"
		"  -z <pem number>: PEM connected to host\n"
		"  -b <burst size>: Initial number of buffers submitted to the DMA\n"
		"  -d <done count>: Min numbers of completions to wait for before resubmission\n"
		"  -p: Performance test\n"
		"  -t <num>: Number of pointers per instruction (Default is 1)\n"
		"  --mps=<size>: Max payload size of PCIe trascation \n"
		"  --fifo_mask=<mask>: FIFO size mask of DPI DMA engines \n"
		"  --mrrs<size>: Max PCIe read request size\n", prgname);
}

/* Parse the argument given in the command line of the application */
static int parse_app_args(int argc, char **argv)
{
	int opt, ret, opt_idx;
	char **argvopt;
	char *prgname = argv[0];

	static struct option lgopts[] = {
		{ "mps", 1, 0, 0},
		{ "mrrs", 1, 0, 0},
		{ "fifo_mask", 1, 0, 0},
		{0, 0, 0, 0 },
	};

	argvopt = argv;

	while ((opt = getopt_long(argc, argvopt, "r:s:l:m:i:pb:z:t:d:", lgopts, &opt_idx)) != EOF) {

		switch (opt) {
		/* portmask */
		case 'r':
			raddr = dpi_parse_addr(optarg);
			if ((long)raddr == 0) {
				printf("invalid remote address\n");
				dpi_usage(prgname);
				return -1;
			}
			printf("raddr: 0x%lx\n", raddr);
			break;
		case 's':
			data_size = dpi_parse_addr(optarg);
			if ((long)data_size == 0) {
				printf("invalid data Size\n");
				dpi_usage(prgname);
				return -1;
			}
			printf("data_size: 0x%lx\n", data_size);
			break;
		case 'l':
			laddr = dpi_parse_addr(optarg);
			if ((long)laddr < 0) {
				printf("invalid local address\n");
				dpi_usage(prgname);
				return -1;
			}
			break;
		case 'm':
			mode = atoi(optarg);
			printf("Mode: %d\n", mode);
			break;
		case 'i':
			n_iter = atoi(optarg);
			break;
		case 'z':
			pem_id = atoi(optarg);
			if (pem_id)
				pem_id = 1;
			break;
		case 'b':
			burst_size = atoi(optarg);
			printf("Burst size:: %d\n", burst_size);
			break;
		case 'd':
			done_count = atoi(optarg);
			break;
		case 't':
			ptrs_per_instr = atoi(optarg);
			printf("Pointers per instr: %d\n", ptrs_per_instr);
			break;
		case 0: /* long options */
			if (!strcmp(lgopts[opt_idx].name, "mps")) {
				mps = atoi(optarg);
				if (mps < DPI_MIN_MPS || mps > DPI_MAX_MPS)
					rte_exit(EXIT_FAILURE, "Invalid max payload size param");
			}
			if (!strcmp(lgopts[opt_idx].name, "mrrs")) {
				mrrs = atoi(optarg);
				if (mrrs < DPI_MIN_MRRS || mrrs > DPI_MAX_MRRS)
					rte_exit(EXIT_FAILURE, "Invalid max read req size param");
			}
			if (!strcmp(lgopts[opt_idx].name, "fifo_mask"))
				fifo_mask = atoll(optarg);
			break;
		default:
			dpi_usage(prgname);
			return -1;
		}
	}

	if (optind >= 0)
		argv[optind-1] = prgname;

	ret = optind-1;
	optind = 1; /* reset getopt lib */
	return ret;
}

int main(int argc, char **argv)
{
	struct dpi_mps_mrrs_cfg mcfg = { 0 };
	struct dpi_engine_cfg ecfg = { 0 };
	struct rte_dma_conf dev_conf;
	unsigned int lcore_id;
	int dpi_file = 0;
	int ret, i;

	total_dma_cnt = 0;
	pem_id = 0;

	ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Invalid EAL arguments\n");
	argc -= ret;
	argv += ret;

	/* parse application arguments (after the EAL ones) */
	ret = parse_app_args(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Invalid App arguments\n");

	nb_dma_devs = rte_dma_count_avail();
	if (nb_dma_devs == 0)
		rte_exit(EXIT_FAILURE, "No dmadevs found\n");

	force_quit = false;
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGUSR1, signal_handler);

	dpi_file = open(dpi_node, O_RDWR);
	if (dpi_file < 0)
		rte_exit(EXIT_FAILURE, "Failed to open dpi pf mode\n");

	mcfg.mps = mps;
	mcfg.mrrs = mrrs;
	ecfg.fifo_mask = fifo_mask;
	ecfg.update_molr = 0;

	if (ioctl(dpi_file, DPI_MPS_MRRS_CFG, &mcfg)) {
		printf("Failed to set MPS & MRRS parameters\n");
		goto err_dpi_cfg;
	}
	if (ioctl(dpi_file, DPI_ENGINE_CFG, &ecfg)) {
		printf("Failed to configure DPI Engine FIFO sizes\n");
		goto err_dpi_cfg;
	}

	/* convert to number of cycles */
	timer_period *= rte_get_timer_hz();

	printf("%d dmadevs detected\n", nb_dma_devs);

	if (burst_size < done_count || burst_size > 256)
		rte_exit(EXIT_FAILURE, "Burst_size must be between %u to 256\n", done_count);

	/* Configure dmadev ports */
	for (i = 0; i < nb_dma_devs; i++) {
		dpi_test[i].dev_id = i;
		rte_dma_info_get(i, &dpi_test[i].dev_info);
		if (ptrs_per_instr > dpi_test[i].dev_info.max_sges) {
			printf("Max pointers can be only %d\n", dpi_test[i].dev_info.max_sges);
			goto err_max_segs;
		}
		dev_conf.nb_vchans = 1;
		dev_conf.enable_silent = 0;
		ret = rte_dma_configure(i, &dev_conf);
		if (ret)
			rte_exit(EXIT_FAILURE, "Unable to configure DPIVF %d\n", i);
		printf("dmadev %d configured successfully\n", i);
	}

	/* launch per-lcore init on every lcore */
	rte_eal_mp_remote_launch(dma_perf_test, NULL, SKIP_MAIN);
	RTE_LCORE_FOREACH_WORKER(lcore_id) {
		if (rte_eal_wait_lcore(lcore_id) < 0) {
			ret = -1;
			break;
		}
	}

err_max_segs:
	for (i = 0; i < nb_dma_devs; i++) {
		rte_dma_stop(i);
		if (rte_dma_close(i))
			printf("Dev close failed for %d device\n", i);
	}

err_dpi_cfg:
	if (dpi_file)
		close(dpi_file);

	rte_eal_cleanup();
	return ret;
}
