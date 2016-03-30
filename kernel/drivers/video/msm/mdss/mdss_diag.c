/* drivers/video/msm/mdss/mdss_diag.c  (Display Driver)
 *
 * Copyright (C) 2014 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

#include <sharp/shdisp_dsi.h>
#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include <linux/iopoll.h>

#define MDSS_DIAG_MIPI_CHECK_AMP_OFF   0x0580
#define MDSS_DIAG_WAIT_1FRAME_US       (16666)

struct mdss_panel_data     * mdss_diag_panel_data;
struct mdss_dsi_ctrl_pdata * mdss_diag_dsi_ctrl;

static uint8_t mdss_diag_mipi_check_amp_data;
static uint8_t mdss_diag_mipi_check_rec_sens_data;
static int mdss_diag_mipi_check_exec_state = false;

static int mdss_diag_mipi_check_exec(uint8_t flame_cnt, uint8_t amp, uint8_t sensitiv);
static int mdss_diag_mipi_check_manual(struct mdp_mipi_check_param * mipi_check_param);
static int mdss_diag_mipi_check_auto(struct mdp_mipi_check_param * mipi_check_param);
static void mdss_diag_mipi_check_set_param(uint8_t amp, uint8_t sensitiv);
static void mdss_diag_mipi_check_get_param(uint8_t *amp, uint8_t *sensitiv);
static int mdss_diag_mipi_check_test(uint8_t flame_cnt);
static int mdss_diag_mipi_check_test_video(uint8_t flame_cnt);
static int mdss_diag_mipi_check_test_cmd(uint8_t flame_cnt);
static int mdss_diag_read_sensitiv(uint8_t *read_data);
static int mdss_diag_dsi_cmd_bta_sw_trigger(struct mdss_panel_data *pdata);
static int mdss_diag_dsi_ack_err_status(struct mdss_dsi_ctrl_pdata *ctrl);
static void mdss_diag_mipi_check_result_convert(
			uint8_t befoe[MDSS_MIPICHK_SENSITIV_NUM][MDSS_MIPICHK_AMP_NUM],
			struct mdp_mipi_check_param * mipi_check_param);
extern void mdss_dsi_err_intr_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, u32 mask,int enable);
extern void mdss_shdisp_video_transfer_ctrl(int onoff);
extern int mdss_shdisp_host_dsi_rx(struct shdisp_dsi_cmd_desc * cmds,
		unsigned char * rx_data, int rx_size);
extern int mdss_shdisp_host_dsi_tx(int commit,
		struct shdisp_dsi_cmd_desc * shdisp_cmds, int size);


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_check_get_exec_state(void)
{
	return mdss_diag_mipi_check_exec_state;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
extern struct fb_info* mdss_fb_get_fbinfo(int);
static inline struct mdss_mdp_ctl* mdss_diag_get_mdpctrl(int fbinx)
{
	return ((struct mdss_overlay_private*)((struct msm_fb_data_type*)mdss_fb_get_fbinfo(fbinx)->par)->mdp.private1)->ctl;
}
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_check(struct mdp_mipi_check_param * mipi_check_param,
			 struct mdss_panel_data *pdata)
{
	int ret;
	u32 isr;

	mdss_diag_panel_data = pdata;
	mdss_diag_dsi_ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);

	pr_debug("%s: called\n", __func__);

	if (!mdss_diag_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_diag_dsi_ctrl=0x%p", __func__, mdss_diag_dsi_ctrl);
		return -ENXIO;
	}

#ifndef SHDISP_DET_DSI_MIPI_ERROR
	/* disable dsi error interrupt */
	mdss_dsi_err_intr_ctrl(mdss_diag_dsi_ctrl, DSI_INTR_ERROR_MASK, 0);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */
	mdss_diag_mipi_check_exec_state = true;

	mdss_diag_dsi_cmd_bta_sw_trigger(mdss_diag_panel_data);

	mdss_diag_mipi_check_get_param(&mdss_diag_mipi_check_amp_data, &mdss_diag_mipi_check_rec_sens_data);

	if (mdss_diag_dsi_ctrl->panel_mode == DSI_VIDEO_MODE) {
		mdss_shdisp_video_transfer_ctrl(false);
	}

	if (mipi_check_param->mode == MDSS_MIPICHK_MANUAL) {
		ret = mdss_diag_mipi_check_manual(mipi_check_param);
	} else if (mipi_check_param->mode == MDSS_MIPICHK_AUTO) {
		ret = mdss_diag_mipi_check_auto(mipi_check_param);
	} else {
		pr_err("%s:mode=%d\n", __func__, mipi_check_param->mode);
		return -EINVAL;
	}

	mdss_diag_mipi_check_set_param(mdss_diag_mipi_check_amp_data, mdss_diag_mipi_check_rec_sens_data);

	if (mdss_diag_dsi_ctrl->panel_mode == DSI_VIDEO_MODE) {
		mdss_shdisp_video_transfer_ctrl(true);
	}

	mdss_diag_mipi_check_exec_state = false;

	isr = MIPI_INP(mdss_diag_dsi_ctrl->ctrl_base + 0x0110);/* DSI_INTR_CTRL */
	MIPI_OUTP(mdss_diag_dsi_ctrl->ctrl_base + 0x0110, isr);

#ifndef SHDISP_DET_DSI_MIPI_ERROR
	/* enable dsi error interrupt */
	mdss_dsi_err_intr_ctrl(mdss_diag_dsi_ctrl, DSI_INTR_ERROR_MASK, 1);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

	pr_debug("%s: end", __func__);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_manual(struct mdp_mipi_check_param * mipi_check_param)
{
	int ret = 0;

	pr_debug("%s: called\n", __func__);

	if ((mipi_check_param->amp & ~0x07) != 0) {
		pr_err("LCDERR: %s AMP=0x%X Out of range", __func__, mipi_check_param->amp);
		return -ENXIO;
	}

	if ((mipi_check_param->sensitiv & ~0x03) != 0) {
		pr_err("LCDERR: %s SENSITIV=0x%X Out of range", __func__, mipi_check_param->sensitiv);
		return -ENXIO;
	}

	ret = mdss_diag_mipi_check_exec(mipi_check_param->flame_cnt, mipi_check_param->amp, mipi_check_param->sensitiv);

	mipi_check_param->result[0] = ret;

	pr_debug("%s: end", __func__);

	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_auto(struct mdp_mipi_check_param * mipi_check_param)
{
	int ret = 0;
	int ret2 = 0;
	uint8_t i,j;
	uint8_t result_temp[MDSS_MIPICHK_SENSITIV_NUM][MDSS_MIPICHK_AMP_NUM]={{0}};
	uint8_t set_flame       = 0x01;
	uint8_t max_amp         = 0x07;
	uint8_t max_sensitiv    = 0x03;

	pr_debug("%s: called\n", __func__);


	for (i = 0; i < MDSS_MIPICHK_SENSITIV_NUM; i++) {
		if (i <4) {
			for (j = 0; j < MDSS_MIPICHK_AMP_NUM; j++) {
				ret = mdss_diag_mipi_check_exec(mipi_check_param->flame_cnt, j, i);
				if (ret == MDSS_MIPICHK_RESULT_NG) {
					ret2 = mdss_diag_mipi_check_exec(set_flame, max_amp, max_sensitiv);
					if (ret2 == MDSS_MIPICHK_RESULT_NG) {
						pr_err("LCDERR: %s mdss_diag_mipi_check_exec ret=%d", __func__, ret);
					}
				}
				result_temp[i][j] = ret;
			}
		}
	}

	mdss_diag_mipi_check_result_convert(result_temp, mipi_check_param);

	pr_debug("%s: end", __func__);

	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_diag_mipi_check_result_convert(
			uint8_t befoe_result[MDSS_MIPICHK_SENSITIV_NUM][MDSS_MIPICHK_AMP_NUM],
			struct mdp_mipi_check_param * mipi_check_param)
{
	uint8_t i,j,x,y;
	uint8_t after_result[MDSS_MIPICHK_SENSITIV_NUM] = {0};

	for (j = 0; j < MDSS_MIPICHK_AMP_NUM; j++) {
		for (i = 0; i < MDSS_MIPICHK_SENSITIV_NUM; i++) {
			if(befoe_result[i][j] == MDSS_MIPICHK_RESULT_OK) {
				x = j * 2;
				if (i >= 8) {
					y = i - 8;
					x++;
				} else {
					y = i;
				}
				after_result[x] |= (1 << (7-y));
			}
		}
	}
	memcpy(mipi_check_param->result, after_result, sizeof(after_result));
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_exec(uint8_t flame_cnt, uint8_t amp, uint8_t sensitiv)
{
	int ret = 0;
	u32 amp_reg;
	u32 amp_tmp;
	uint8_t set_sensitiv = 0;

	uint8_t amp_tbl[MDSS_MIPICHK_AMP_NUM] = {
	    0x00,
	    0x01,
	    0x02,
	    0x03,
	    0x04,
	    0x05,
	    0x06,
	    0x07
	};

	pr_debug("%s: called flame_cnt=0x%02X amp=0x%02X sensitiv=0x%02X\n", __func__, flame_cnt, amp, sensitiv);

	amp_tmp = amp_tbl[amp];
	amp_reg = (amp_tmp << 1) |1;

	mdss_diag_mipi_check_set_param(amp_reg, set_sensitiv);

	ret = mdss_diag_mipi_check_test(flame_cnt);

	pr_debug("%s: end ret=%d \n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_check_set_param(uint8_t amp, uint8_t sensitiv)
{
	char payload_sensitiv1[1][5] = {
	    {0xFF, 0xAA,0x55,0x25,0x01},
	};
	char payload_sensitiv2[1][2] = {
	    {0x6F, 0x03},
	};
	char payload_sensitiv3[1][2] = {
	    {0xF8, 0x00},
	};

	struct shdisp_dsi_cmd_desc cmds_sensitiv[] = {
		{SHDISP_DTYPE_DCS_LWRITE, 5, payload_sensitiv1[0], 0},
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_sensitiv2[0], 0},
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_sensitiv3[0], 0},
	};

	MIPI_OUTP((mdss_diag_dsi_ctrl->ctrl_base) + MDSS_DIAG_MIPI_CHECK_AMP_OFF, amp);
	wmb();

	sensitiv |= (mdss_diag_mipi_check_rec_sens_data & 0xFC);
	payload_sensitiv3[0][1] = sensitiv;

	mdss_shdisp_host_dsi_tx(1, cmds_sensitiv, ARRAY_SIZE(cmds_sensitiv));
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_check_get_param(uint8_t *amp, uint8_t *sensitiv)
{
	int ret = 0;
	*amp = MIPI_INP((mdss_diag_dsi_ctrl->ctrl_base) + MDSS_DIAG_MIPI_CHECK_AMP_OFF);

	ret = mdss_diag_read_sensitiv(sensitiv);

	pr_debug("%s: amp=0x%02X sensitiv=0x%02X\n", __func__, *amp, *sensitiv);

}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test(uint8_t flame_cnt)
{
	int ret = 0;
	char mode;

	mode = mdss_diag_dsi_ctrl->panel_mode;

	if (mode == DSI_VIDEO_MODE) {
		ret = mdss_diag_mipi_check_test_video(flame_cnt); 
	} else if (mode == DSI_CMD_MODE) {
		ret = mdss_diag_mipi_check_test_cmd(flame_cnt);
	} else {
		pr_err("LCDERR: %s paneltype=%d\n", __func__, mode);
		ret = -EINVAL;
	}

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test_video(uint8_t flame_cnt)
{
	int ret = 0;
	uint32_t sleep;

	sleep = flame_cnt * MDSS_DIAG_WAIT_1FRAME_US;
	pr_debug("%s: flame=%d sleep time=%d\n", __func__, flame_cnt, sleep);

	mdss_shdisp_video_transfer_ctrl(true);

	usleep(sleep);

	ret = mdss_diag_dsi_cmd_bta_sw_trigger(mdss_diag_panel_data);
	if (ret) {
		ret = MDSS_MIPICHK_RESULT_NG;
	} else {
		ret = MDSS_MIPICHK_RESULT_OK;
	}

	mdss_shdisp_video_transfer_ctrl(false);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test_cmd(uint8_t flame_cnt)
{
	int ret = 0;
	int ret2 = 0;
	int i;
	struct mdss_mdp_ctl *pctl;

	pctl = mdss_diag_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] mdpctrl is NULL.\n", __func__);
		return MDSS_MIPICHK_RESULT_NG;
	}

	if (!pctl->display_fnc) {
		pr_err("LCDERR:[%s] display_fnc is NULL.\n", __func__);
		return MDSS_MIPICHK_RESULT_NG;
	}

	for (i = 0; i < flame_cnt; i++) {
		mutex_lock(&pctl->lock);
		mdss_mdp_ctl_perf_set_transaction_status(pctl, PERF_SW_COMMIT_STATE, PERF_STATUS_BUSY);

		mdss_mdp_ctl_perf_update_ctl(pctl, 1);

		if (pctl->wait_pingpong) {
			ret2 = pctl->wait_pingpong(pctl, NULL);
			if(ret2){
				pr_err("LCDERR:[%s] failed to wait_pingpong(). (ret=%d)\n", __func__, ret2);
			}
		}

		ret = pctl->display_fnc(pctl, NULL);
		if (ret) {
			pr_err("LCDERR:[%s] failed to display_fnc(). (ret=%d)\n", __func__, ret);
			mutex_unlock(&pctl->lock);
			return MDSS_MIPICHK_RESULT_NG;
		}
		mutex_unlock(&pctl->lock);
	}

	if (pctl->wait_pingpong) {
		ret2 = pctl->wait_pingpong(pctl, NULL);
		if(ret2){
			pr_err("LCDERR:[%s] failed to wait_pingpong(). (ret=%d)\n", __func__, ret2);
		}
	}

	ret = mdss_diag_dsi_cmd_bta_sw_trigger(mdss_diag_panel_data);
	if (ret) {
		ret = MDSS_MIPICHK_RESULT_NG;
	} else {
		ret = MDSS_MIPICHK_RESULT_OK;
	}

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_read_sensitiv(uint8_t *read_data)
{
	int ret = 0;
	struct shdisp_dsi_cmd_desc cmd[1];
	char cmd_buf[1 + 2];
	char payload_page_ee1[1][5] = {
	    {0xFF, 0xAA,0x55,0x25,0x01},
	};
	char payload_page_ee2[1][2] = {
	    {0x6F, 0x03},
	};

	struct shdisp_dsi_cmd_desc cmds_sensitiv[] = {
		{SHDISP_DTYPE_DCS_LWRITE, 5, payload_page_ee1[0], 0},
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_page_ee2[0], 0},
	};

	mdss_shdisp_host_dsi_tx(1, cmds_sensitiv, ARRAY_SIZE(cmds_sensitiv));

	cmd_buf[0] = 0xF8;
	cmd_buf[1] = 0x00;

	cmd[0].dtype = SHDISP_DTYPE_DCS_READ;
	cmd[0].wait = 0x00;
	cmd[0].dlen = 1;
	cmd[0].payload = cmd_buf;

	ret = mdss_shdisp_host_dsi_rx(cmd, read_data, 1);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_dsi_cmd_bta_sw_trigger(struct mdss_panel_data *pdata)
{
	u32 status;
	int timeout_us = 35000, ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);

	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x098, 0x01);	/* trigger */
	wmb();

	/* Check for CMD_MODE_DMA_BUSY */
	if (readl_poll_timeout(((ctrl_pdata->ctrl_base) + 0x0008),
				status, ((status & 0x0010) == 0),
				0, timeout_us))
	{
		pr_info("%s: DSI status=%x failed\n", __func__, status);
		return -EIO;
	}

	ret = mdss_diag_dsi_ack_err_status(ctrl_pdata);

	pr_debug("%s: BTA done, status = %d\n", __func__, status);
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_dsi_ack_err_status(struct mdss_dsi_ctrl_pdata *ctrl)
{
	u32 status;
	unsigned char *base;
	u32 ack = 0x10000000;

	base = ctrl->ctrl_base;

	status = MIPI_INP(base + 0x0068);/* DSI_ACK_ERR_STATUS */

	if (status) {
		MIPI_OUTP(base + 0x0068, status);
		/* Writing of an extra 0 needed to clear error bits */
		MIPI_OUTP(base + 0x0068, 0);

		status &= ~(ack);
		if(status){
			pr_err("%s: status=%x\n", __func__, status);
			return -EIO;
		}
	}
	return 0;
}


