/*
 * This file is provided under a GPLv2 license. When using or redistributing
 *   this file, you may do so under that license.
 *
 *   GPL LICENSE SUMMARY
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundataion.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Contact Information:
 * Karl Kao <karl.kao@gmail.com>
 */

#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/ntb.h>

#include "ntb_hw_plx.h"

#define NTB_NAME	"ntb_hw_plx"
#define NTB_DESC	"PLX PCI-E Non-Transparent Bridge Driver"
#define NTB_VER		"1.0"

MODULE_DESCRIPTION(NTB_DESC);
MODULE_VERSION(NTB_VER);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Karl Kao");

static const struct file_operations plxntb_debugfs_info;
static struct dentry *debugfs_dir;

enum switch_port {
	UPSTREAM_PORT = 0,
	NTB_PORT
};

enum ntb_rid_side {
	PLXNTB_VIRTUAL_SIDE = 0,
	PLXNTB_LINK_SIDE
};

enum ntb_bar_number {
	NTB_BAR_NUM_ZERO = 0,
	NTB_BAR_NUM_ONE = 1,
	NTB_BAR_NUM_TWO = 2,
	NTB_BAR_NUM_THREE = 3,
	NTB_BAR_NUM_FOUR = 4
};

static inline void iowrite32or(u32 val, void __iomem *mmio)
{
	u32 data;

	data = ioread32(mmio);
	data |= val;
	iowrite32(data, mmio);
}

static inline void iowrite32and(u32 val, void __iomem *mmio)
{
	u32 data;

	data = ioread32(mmio);
	data &= val;
	iowrite32(data, mmio);
}

static inline void ndev_reset_unsafe_flags(struct plx_ntb_dev *ndev)
{
	ndev->unsafe_flags = 0;
	ndev->unsafe_flags_ignore = 0;
}

static inline int ndev_ignore_unsafe(struct plx_ntb_dev *ndev,
				     unsigned long flag)
{
	flag &= ndev->unsafe_flags;
	ndev->unsafe_flags_ignore |= flag;

	return !!flag;
}

static inline int ndev_is_unsafe(struct plx_ntb_dev *ndev,
				 unsigned long flag)
{
	return !!(flag & ndev->unsafe_flags & ~ndev->unsafe_flags_ignore);
}

static int ndev_spad_addr(struct plx_ntb_dev *ndev, int idx,
			  phys_addr_t *spad_addr, phys_addr_t reg_addr)
{
	if (ndev_is_unsafe(ndev, NTB_UNSAFE_SPAD))
		pr_warn_once("%s: NTB unsafe scratchpad access", __func__);

	if (idx < 0 || idx >= ndev->spad_count)
		return -EINVAL;

	if (spad_addr) {
		*spad_addr = reg_addr + (idx << PLXNTB_SPAD_SHIFT);
		dev_dbg(&ndev->ntb.pdev->dev, "Spad addr 0x%llx\n", *spad_addr);
	}

	return 0;
}

static int ndev_spad_write(struct plx_ntb_dev *ndev, int idx, u32 val,
			   void __iomem *mmio)
{
	if (ndev_is_unsafe(ndev, NTB_UNSAFE_SPAD))
		pr_warn_once("%s: NTB unsafe scratchpad access", __func__);

	if (idx < 0 || idx >= ndev->spad_count)
		return -EINVAL;

	iowrite32(val, mmio + (idx << PLXNTB_SPAD_SHIFT));

	return 0;
}

static u32 ndev_spad_read(struct plx_ntb_dev *ndev, int idx, void __iomem *mmio)
{
	if (ndev_is_unsafe(ndev, NTB_UNSAFE_SPAD))
		pr_warn_once("%s: NTB unsafe scratchpad access", __func__);

	if (idx < 0 || idx >= ndev->spad_count)
		return -EINVAL;

	return ioread32(mmio + (idx << PLXNTB_SPAD_SHIFT));
}

static int ndev_db_write_mask(struct plx_ntb_dev *ndev, u64 db_bits,
			      void __iomem *mmio)
{
	unsigned long irqflags;

	if (ndev_is_unsafe(ndev, NTB_UNSAFE_DB))
		pr_warn_once("%s: NTB unsafe doorbell access", __func__);

	if (db_bits & ~ndev->db_valid_mask)
		return -EINVAL;

	db_bits &= PLXNTB_VIRTUAL_DOORBELL_MASK;

	spin_lock_irqsave(&ndev->db_mask_lock, irqflags);
	ndev->db_mask |= db_bits;
	iowrite32((u32)ndev->db_mask, mmio);
	spin_unlock_irqrestore(&ndev->db_mask_lock, irqflags);

	return 0;
}

static int ndev_db_write(struct plx_ntb_dev *ndev, u64 db_bits,
			 void __iomem *mmio)
{
	if (ndev_is_unsafe(ndev, NTB_UNSAFE_DB))
		pr_warn_once("%s: NTB unsafe doorbell access", __func__);

	if (db_bits & ~ndev->db_valid_mask)
		return -EINVAL;

	db_bits &= PLXNTB_VIRTUAL_DOORBELL_MASK;

	iowrite32((u32)db_bits, mmio);

	return 0;
}

static u64 ndev_db_read(struct plx_ntb_dev *ndev, void __iomem *mmio)
{
	if (ndev_is_unsafe(ndev, NTB_UNSAFE_DB))
		pr_warn_once("%s: NTB unsafe doorbell access", __func__);

	return (u64)(ioread32(mmio) & PLXNTB_VIRTUAL_DOORBELL_MASK);
}

static u64 ndev_vec_mask(struct plx_ntb_dev *ndev, int db_vector)
{
	u64 shift, mask;

	shift = ndev->db_vec_shift;
	mask = BIT_ULL(shift) - 1;

	return mask << (shift * db_vector);
}

static inline int ndev_mw_to_bar_num(struct plx_ntb_dev *ndev, int idx)
{
	if (idx < 0 || idx > ndev->mw_count)
		return -EINVAL;

	return ndev->switch_cfg->mw_bar[idx];
}

static inline int ndev_is_link_state_changed(struct plx_ntb_dev *ndev)
{
	u32 __iomem *virt_reg, reg_val;

	virt_reg = ndev->local_mmio.virtual_pcireg;

	/* get link speed and width */
	reg_val = ioread32(&virt_reg[PLXNTB_LINK_STATUS_AND_CONTROL_REG]);
	reg_val &= PLXNTB_LINK_STATUS_SPEED_WIDTH_MASK;

	reg_val >>= PLXNTB_LINK_STATUS_SHIFT;

	if (ndev->link_speed_width == reg_val)
		return 0;

	ndev->link_speed_width = reg_val;

	return 1;
}

static void ndev_station_port_parse(enum switch_port_num port,
				    enum switch_port_num *station_port)
{
	if (port < PLXNTB_PORT8)
		*station_port = PLXNTB_PORT0;
	else if (port < PLXNTB_PORT16)
		*station_port = PLXNTB_PORT8;
	else
		*station_port = PLXNTB_PORT16;
}

static u32 ndev_get_port_base_reg(enum switch_port_num port)
{
	u32 port_basereg;

	switch (port) {
	case PLXNTB_PORT0:
		return PLXNTB_PORT0_BASE_REG;
	case PLXNTB_PORT1:
		return PLXNTB_PORT1_BASE_REG;
	case PLXNTB_PORT8:
		return PLXNTB_PORT8_BASE_REG;
	case PLXNTB_PORT9:
		return PLXNTB_PORT9_BASE_REG;
	case PLXNTB_PORT16:
		return PLXNTB_PORT16_BASE_REG;
	case PLXNTB_PORT21:
		return PLXNTB_PORT21_BASE_REG;

	default:
		port_basereg = PLXNTB_PORT0_BASE_REG +
			       ((PLXNTB_PORT_BASE_OFFSET * port) / sizeof(u32));

		return port_basereg;
	}
}

static inline int ndev_link_is_up(struct plx_ntb_dev *ndev)
{
	return ndev->link_speed_width & PLXNTB_LINK_WIDTH_MASK;
}

static u64 plxntb_ops_link_is_up(struct ntb_dev *ntb,
				 enum ntb_speed *speed,
				 enum ntb_width *width)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);

	if (ndev_link_is_up(ndev)) {
		if (speed)
			*speed = PLXNTB_LINK_SPEED(ndev->link_speed_width);
		if (width)
			*width = PLXNTB_LINK_WIDTH(ndev->link_speed_width);

		return 1;
	}

	if (speed)
		*speed = NTB_SPEED_NONE;
	if (width)
		*width = NTB_WIDTH_NONE;

	return 0;
}

static int plxntb_ops_link_enable(struct ntb_dev *ntb,
				  enum ntb_speed max_speed,
				  enum ntb_width max_width)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *base_reg, port_basereg, control;
	enum switch_port_num station_port;

	base_reg = ndev->local_mmio.base_pcireg;

	/* look up station port */
	ndev_station_port_parse(ndev->ntb_port, &station_port);

	/* set control base as per station port */
	port_basereg = ndev_get_port_base_reg(station_port);
	base_reg = &base_reg[port_basereg];

	/* generate control bit as per station port */
	control = ndev->ntb_port;

	if (station_port)
		control = ndev->ntb_port % station_port;

	control = (PLXNTB_PORT_DISABLE_BIT << control);

	/* enable NTB port */
	iowrite32and(~control, &base_reg[PLXNTB_PORT_CONTROL]);

	dev_dbg(&ntb->pdev->dev,
		"NTB Port:%d, Station Port:%d, Ctrl:0x%08x, Reg:0x%08x\n",
		ndev->ntb_port, station_port, control,
		ioread32(&base_reg[PLXNTB_PORT_CONTROL]));

	ndev->port_state = NTB_PORT_ENABLED;

	dev_dbg(&ntb->pdev->dev,
		"Enabling NTB link with max_speed %d max_width %d\n",
		max_speed, max_width);

	if (max_speed != NTB_SPEED_AUTO)
		dev_dbg(&ntb->pdev->dev, "ignoring max_speed %d\n", max_speed);

	if (max_width != NTB_WIDTH_AUTO)
		dev_dbg(&ntb->pdev->dev, "ignoring max_width %d\n", max_width);

	return 0;
}

static int plxntb_ops_link_disable(struct ntb_dev *ntb)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *base_reg, port_basereg, control;
	enum switch_port_num station_port;

	base_reg = ndev->local_mmio.base_pcireg;

	/* look up station port */
	ndev_station_port_parse(ndev->ntb_port, &station_port);

	/* set control base as per station port */
	port_basereg = ndev_get_port_base_reg(station_port);
	base_reg = &base_reg[port_basereg];

	/* generate control bit as per station port */
	control = ndev->ntb_port;

	if (station_port)
		control = ndev->ntb_port % station_port;

	control = (PLXNTB_PORT_DISABLE_BIT << control);

	/* disable NTB port */
	iowrite32or(control, &base_reg[PLXNTB_PORT_CONTROL]);

	dev_dbg(&ntb->pdev->dev,
		"NTB Port:%d, Station Port:%d, Ctrl:0x%08x, Reg:0x%08x\n",
		ndev->ntb_port, station_port, control,
		ioread32(&base_reg[PLXNTB_PORT_CONTROL]));

	ndev->port_state = NTB_PORT_DISABLED;

	return 0;
}

static int plxntb_ops_mw_count(struct ntb_dev *ntb, int pidx)
{
	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	return ntb_ndev(ntb)->mw_count;
}

static int plxntb_ops_mw_get_align(struct ntb_dev *ntb, int pidx, int idx,
				   resource_size_t *addr_align,
				   resource_size_t *size_align,
				   resource_size_t *size_max)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	resource_size_t res_size;
	int bar;

	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	bar = ndev_mw_to_bar_num(ndev, idx);
	if (bar < NTB_BAR_NUM_ZERO)
		return bar;

	if (addr_align) {
		res_size = pci_resource_len(ndev->ntb.pdev, bar);

		if (res_size > SZ_1M)
			*addr_align = SZ_1M;
		else
			*addr_align = res_size;
	}

	if (size_align)
		*size_align = 1;

	if (size_max)
		*size_max = pci_resource_len(ndev->ntb.pdev, bar);

	return 0;
}

static int plxntb_ops_mw_set_trans(struct ntb_dev *ntb, int pidx, int idx,
				   dma_addr_t addr, resource_size_t size)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *link_reg, mask32;
	u64 mask64;
	int bar;

	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	/* size can be 0 to be as trans clear */
	if (size < 0)
		return -EINVAL;

	bar = ndev_mw_to_bar_num(ndev, idx);
	if (bar < NTB_BAR_NUM_ZERO)
		return bar;

	link_reg = ndev->local_mmio.link_pcireg;

	switch (bar) {
	case NTB_BAR_NUM_TWO:
		if (ndev->bar2_res.base64) {
			/* Set BAR2/BAR3 translation registers */
			iowrite32(addr,
			  &link_reg[PLXNTB_BAR2_ADDRESS_TRANSLATION_LOWER_REG]);

			iowrite32(addr >> 32,
			  &link_reg[PLXNTB_BAR3_ADDRESS_TRANSLATION_UPPER_REG]);

			/* Set BAR2/BAR3 setup register according to size */
			mask64 = size - 1;
			mask64 = ~mask64;
			mask64 |= PLXNTB_BAR_64_BIT;
			mask64 |= PLXNTB_BAR_PREFETCHABLE;

			iowrite32((u32)mask64,
				  &link_reg[PLXNTB_LINK_BAR2_SETUP_REG]);

			iowrite32((u32)(mask64 >> 32),
				  &link_reg[PLXNTB_LINK_BAR3_SETUP_REG]);
		} else {
			/* Set BAR2 translation registers */
			iowrite32(addr,
			  &link_reg[PLXNTB_BAR2_ADDRESS_TRANSLATION_LOWER_REG]);

			/* Set BAR2 setup register according to size */
			mask32 = size - 1;
			mask32 = ~mask32;
			mask32 |= PLXNTB_BAR_PREFETCHABLE;

			iowrite32((u32)mask32,
				  &link_reg[PLXNTB_LINK_BAR2_SETUP_REG]);
		}
		break;
	case NTB_BAR_NUM_FOUR:
		if (ndev->bar4_res.base64) {
			/* Set BAR4/BAR5 translation registers */
			iowrite32(addr,
			  &link_reg[PLXNTB_BAR4_ADDRESS_TRANSLATION_LOWER_REG]);

			iowrite32(addr >> 32,
			  &link_reg[PLXNTB_BAR5_ADDRESS_TRANSLATION_UPPER_REG]);

			/* Set BAR4/BAR5 setup register according to size */
			mask64 = size - 1;
			mask64 = ~mask64;
			mask64 |= PLXNTB_BAR_64_BIT;
			mask64 |= PLXNTB_BAR_PREFETCHABLE;

			iowrite32((u32)mask64,
				  &link_reg[PLXNTB_LINK_BAR4_SETUP_REG]);

			iowrite32((u32)(mask64 >> 32),
				  &link_reg[PLXNTB_LINK_BAR5_SETUP_REG]);
		} else {
			/* Set BAR4 translation registers */
			iowrite32(addr,
			  &link_reg[PLXNTB_BAR4_ADDRESS_TRANSLATION_LOWER_REG]);

			/* Set BAR4 setup register according to size */
			mask32 = size - 1;
			mask32 = ~mask32;
			mask32 |= PLXNTB_BAR_PREFETCHABLE;

			iowrite32((u32)mask32,
				  &link_reg[PLXNTB_LINK_BAR4_SETUP_REG]);
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int plxntb_ops_peer_mw_count(struct ntb_dev *ntb)
{
	/* the same numnber as inbound (local) memory windows */
	return ntb_ndev(ntb)->mw_count;
}

static int plxntb_ops_peer_mw_get_addr(struct ntb_dev *ntb, int idx,
				       phys_addr_t *base, resource_size_t *size)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	resource_size_t res_size;
	int bar;

	bar = ndev_mw_to_bar_num(ndev, idx);
	if (bar < NTB_BAR_NUM_ZERO)
		return -EINVAL;

	if (base)
		*base = pci_resource_start(ndev->ntb.pdev, bar);

	if (size) {
		res_size = pci_resource_len(ndev->ntb.pdev, bar);

		if (res_size > SZ_1M)
			*size = SZ_1M;
		else
			*size = res_size;
	}

	return 0;
}

static int plxntb_ops_db_is_unsafe(struct ntb_dev *ntb)
{
	return ndev_ignore_unsafe(ntb_ndev(ntb), NTB_UNSAFE_DB);
}

static u64 plxntb_ops_db_valid_mask(struct ntb_dev *ntb)
{
	return ntb_ndev(ntb)->db_valid_mask;
}

static int plxntb_ops_db_vector_count(struct ntb_dev *ntb)
{
	return ntb_ndev(ntb)->db_vec_count;
}

static u64 plxntb_ops_db_vector_mask(struct ntb_dev *ntb, int db_vector)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);

	if (db_vector < 0 || db_vector > ndev->db_vec_count)
		return 0;

	return ndev->db_valid_mask & ndev_vec_mask(ndev, db_vector);
}

static u64 plxntb_ops_db_read(struct ntb_dev *ntb)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *virt_reg;

	virt_reg = ndev->local_mmio.virtual_pcireg;

	return ndev_db_read(ndev,
			    &virt_reg[PLXNTB_VIRTUAL_DOORBELL_IRQ_SET_REG]);
}

static int plxntb_ops_db_clear(struct ntb_dev *ntb, u64 db_bits)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *virt_reg;

	virt_reg = ndev->local_mmio.virtual_pcireg;

	return ndev_db_write(ndev, db_bits,
			     &virt_reg[PLXNTB_VIRTUAL_DOORBELL_IRQ_CLEAR_REG]);
}

static int plxntb_ops_db_set_mask(struct ntb_dev *ntb, u64 db_bits)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *virt_reg;

	virt_reg = ndev->local_mmio.virtual_pcireg;

	return ndev_db_write_mask(ndev, db_bits,
			&virt_reg[PLXNTB_VIRTUAL_DOORBELL_IRQ_MASK_SET_REG]);
}

static int plxntb_ops_db_clear_mask(struct ntb_dev *ntb, u64 db_bits)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *virt_reg;

	virt_reg = ndev->local_mmio.virtual_pcireg;

	return ndev_db_write_mask(ndev, db_bits,
			&virt_reg[PLXNTB_VIRTUAL_DOORBELL_IRQ_MASK_CLEAR_REG]);
}

static int plxntb_ops_peer_db_addr(struct ntb_dev *ntb,
				   phys_addr_t *db_addr,
				   resource_size_t *db_size)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	phys_addr_t base;

	if (ndev_is_unsafe(ndev, NTB_UNSAFE_DB))
		pr_warn_once("%s: NTB unsafe doorbell access", __func__);

	if (db_addr) {
		base = pci_resource_start(ndev->ntb.pdev,
					  ndev->switch_cfg->remote_bar_num);
		*db_addr = base +
			   PLXNTB_VIRTUAL_DOORBELL_IRQ_SET_REG * sizeof(u32);

		dev_dbg(&ndev->ntb.pdev->dev,
			"Peer db addr 0x%llx\n", *db_addr);
	}

	if (db_size) {
		*db_size = sizeof(u16);

		dev_dbg(&ndev->ntb.pdev->dev,
			"Peer db size 0x%llx\n", *db_size);
	}

	return 0;
}

static int plxntb_ops_peer_db_set(struct ntb_dev *ntb, u64 db_bits)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *peer_virt_reg;

	peer_virt_reg = ndev->remote_mmio.virtual_pcireg;

	return ndev_db_write(ndev, db_bits,
			   &peer_virt_reg[PLXNTB_VIRTUAL_DOORBELL_IRQ_SET_REG]);
}

static int plxntb_ops_spad_is_unsafe(struct ntb_dev *ntb)
{
	return ndev_ignore_unsafe(ntb_ndev(ntb), NTB_UNSAFE_SPAD);
}

static int plxntb_ops_spad_count(struct ntb_dev *ntb)
{
	return ntb_ndev(ntb)->spad_count;
}

static u32 plxntb_ops_spad_read(struct ntb_dev *ntb, int idx)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *virt_reg;

	virt_reg = ndev->local_mmio.virtual_pcireg;

	return ndev_spad_read(ndev, idx,
			      &virt_reg[PLXNTB_VIRTUAL_SCRATCHPAD_0_REG]);
}

static int plxntb_ops_spad_write(struct ntb_dev *ntb, int idx, u32 val)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *virt_reg;

	virt_reg = ndev->local_mmio.virtual_pcireg;

	return ndev_spad_write(ndev, idx, val,
			       &virt_reg[PLXNTB_VIRTUAL_SCRATCHPAD_0_REG]);
}

static int plxntb_ops_peer_spad_addr(struct ntb_dev *ntb, int pidx,
				     int idx, phys_addr_t *spad_addr)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	phys_addr_t reg_addr, base;

	base = pci_resource_start(ndev->ntb.pdev,
				  ndev->switch_cfg->remote_bar_num);

	reg_addr = base + PLXNTB_VIRTUAL_SCRATCHPAD_0_REG * sizeof(u32);

	return ndev_spad_addr(ndev, idx, spad_addr, reg_addr);
}

static u32 plxntb_ops_peer_spad_read(struct ntb_dev *ntb, int pidx, int idx)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *peer_virt_reg;

	peer_virt_reg = ndev->remote_mmio.virtual_pcireg;

	return ndev_spad_read(ndev, idx,
			      &peer_virt_reg[PLXNTB_VIRTUAL_SCRATCHPAD_0_REG]);
}

static int plxntb_ops_peer_spad_write(struct ntb_dev *ntb, int pidx,
				      int idx, u32 val)
{
	struct plx_ntb_dev *ndev = ntb_ndev(ntb);
	u32 __iomem *peer_virt_reg;

	peer_virt_reg = ndev->remote_mmio.virtual_pcireg;

	return ndev_spad_write(ndev, idx, val,
			       &peer_virt_reg[PLXNTB_VIRTUAL_SCRATCHPAD_0_REG]);
}

static const struct ntb_dev_ops plx_ntb_ops = {
	.link_is_up		= plxntb_ops_link_is_up,
	.link_enable		= plxntb_ops_link_enable,
	.link_disable		= plxntb_ops_link_disable,
	.mw_count		= plxntb_ops_mw_count,
	.mw_get_align		= plxntb_ops_mw_get_align,
	.mw_set_trans		= plxntb_ops_mw_set_trans,
	.peer_mw_count		= plxntb_ops_peer_mw_count,
	.peer_mw_get_addr	= plxntb_ops_peer_mw_get_addr,
	.db_is_unsafe		= plxntb_ops_db_is_unsafe,
	.db_valid_mask		= plxntb_ops_db_valid_mask,
	.db_vector_count	= plxntb_ops_db_vector_count,
	.db_vector_mask		= plxntb_ops_db_vector_mask,
	.db_read		= plxntb_ops_db_read,
	.db_clear		= plxntb_ops_db_clear,
	.db_set_mask		= plxntb_ops_db_set_mask,
	.db_clear_mask		= plxntb_ops_db_clear_mask,
	.peer_db_addr		= plxntb_ops_peer_db_addr,
	.peer_db_set		= plxntb_ops_peer_db_set,
	.spad_is_unsafe		= plxntb_ops_spad_is_unsafe,
	.spad_count		= plxntb_ops_spad_count,
	.spad_read		= plxntb_ops_spad_read,
	.spad_write		= plxntb_ops_spad_write,
	.peer_spad_addr		= plxntb_ops_peer_spad_addr,
	.peer_spad_read		= plxntb_ops_peer_spad_read,
	.peer_spad_write	= plxntb_ops_peer_spad_write,
};

static void ndev_dump_config_space(struct plx_ntb_dev *ndev,
				   u32 __iomem *mmio, int rows,
				   char *buf_, ssize_t *off_, size_t buf_size_)
{
	int n;
	ssize_t off = *off_;

	for (n = 0; n < rows; n++) {
		off += scnprintf(buf_ + off, buf_size_ - off,
				 "\t0x%p:\t0x%08x  0x%08x  0x%08x  0x%08x\n",
				 &mmio[n * 4],
				 ioread32(&mmio[n * 4]),
				 ioread32(&mmio[n * 4 + 1]),
				 ioread32(&mmio[n * 4 + 2]),
				 ioread32(&mmio[n * 4 + 3]));
	}

	*off_ = off;
}

static void ndev_dump_rid_table(struct plx_ntb_dev *ndev,
				enum ntb_rid_side ntb_side,
				char *buf_, ssize_t *off_, size_t buf_size_)
{
	int n;
	u32 __iomem *rid_table;
	ssize_t off = *off_;

	if (ntb_side == PLXNTB_VIRTUAL_SIDE)
		rid_table = ndev->local_mmio.virtual_rid_lookup_table;
	else
		rid_table = ndev->local_mmio.link_rid_lookup_table;

	for (n = 0; n < PLXNTB_RID_ENTRY_PAIRS / 4; n++) {
		off += scnprintf(buf_ + off, buf_size_ - off,
				 "\t0x%p:\t0x%08x  0x%08x  0x%08x  0x%08x\n",
				 &rid_table[n * 4],
				 ioread32(&rid_table[n * 4]),
				 ioread32(&rid_table[n * 4 + 1]),
				 ioread32(&rid_table[n * 4 + 2]),
				 ioread32(&rid_table[n * 4 + 3]));
	}

	*off_ = off;
}

static ssize_t ndev_debugfs_debug_read(struct file *filp, char __user *ubuf,
				       size_t count, loff_t *offp)
{
	struct plx_ntb_dev *ndev;
	char *buf;
	size_t buf_size;
	ssize_t ret, off;
	union { u64 v64; u32 v32; } u;
	u32 __iomem *virt_reg, __iomem *link_reg;
	u32 __iomem *base_reg, port_basereg;
	enum switch_port_num station_port;
	const size_t MAX_BUF_SIZE = 0x800;

	ndev = filp->private_data;

	buf_size = min(count, MAX_BUF_SIZE);

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	virt_reg = ndev->local_mmio.virtual_pcireg;
	link_reg = ndev->local_mmio.link_pcireg;

	off = 0;

	/* Info device registers */
	base_reg = ndev->local_mmio.base_pcireg;

	/* upstream port */
	off += scnprintf(buf + off, buf_size - off,
			"Upstream Port Register:\n");

	ndev_station_port_parse(ndev->ups_port, &station_port);
	port_basereg = ndev_get_port_base_reg(station_port);
	base_reg = &base_reg[port_basereg];

	u.v32 = ioread32(&base_reg[PLXNTB_PORT_CONTROL]);
	off += scnprintf(buf + off, buf_size - off,
			"\tPort Control\t\t\t(208h): 0x%08x\n", u.v32);

	iowrite32(ndev->ups_port << PLXNTB_RECOVERY_DIAGNOSTIC_PORT_SHIFT,
		  &base_reg[PLXNTB_RECOVERY_DIAGNOSTIC_REG]);

	u.v32 = ioread32(&base_reg[PLXNTB_RECOVERY_DIAGNOSTIC_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tRecovery Diagnostic\t\t(BC4h): 0x%08x\n", u.v32);

	/* NTB port */
	ndev_station_port_parse(ndev->ntb_port, &station_port);
	port_basereg = ndev_get_port_base_reg(station_port);
	base_reg = &base_reg[port_basereg];

	iowrite32(ndev->ntb_port << PLXNTB_RECOVERY_DIAGNOSTIC_PORT_SHIFT,
		  &base_reg[PLXNTB_RECOVERY_DIAGNOSTIC_REG]);

	u.v32 = ioread32(&base_reg[PLXNTB_RECOVERY_DIAGNOSTIC_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tNTB Recovery Diagnostic\t\t(BC4h): 0x%08x\n", u.v32);

	/* Link side registers */
	off += scnprintf(buf + off, buf_size - off,
			"Link Side Register:\n");

	u.v32 = ioread32(&link_reg[PLXNTB_COMMAND_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tDevice Status and Control\t(04h):  0x%08x\n", u.v32);

	u.v32 = ioread32(&link_reg[PLXNTB_LINK_STATUS_AND_CONTROL_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tLink Status and Control\t\t(78h):  0x%08x\n", u.v32);

	u.v32 = ioread32(&link_reg[PLXNTB_LINK_STATUS_AND_CONTROL_2_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tLink Status and Control 2\t(98h):  0x%08x\n", u.v32);

	u.v32 = ioread32(&link_reg[PLXNTB_GEN_FRAMING_ERROR_STATUS_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tGen 3 Framing Error Status\t(724h): 0x%08x\n",
			u.v32);

	u.v32 = ioread32(&link_reg[PLXNTB_LINK_SIDE_BAD_TLP_COUNT_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tBad TLP Count:\t\t\t(FACh): 0x%08x\n", u.v32);

	u.v32 = ioread32(&link_reg[PLXNTB_LINK_SIDE_BAD_DLLP_COUNT_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tBad DLLP Count:\t\t\t(FB0h): 0x%08x\n", u.v32);

	u.v32 = ioread32(&link_reg[PLXNTB_UNCORRECTABLE_ERROR_STATUS_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tUncorrectable Error Status:\t(FB8h): 0x%08x\n",
			u.v32);

	u.v32 = ioread32(&link_reg[PLXNTB_CORRECTABLE_ERROR_STATUS_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tCorrectable Error Status:\t(FC4h): 0x%08x\n", u.v32);

	/* dump config spcae */
	off += scnprintf(buf + off, buf_size - off,
			"Config Space:\n");

	off += scnprintf(buf + off, buf_size - off,
			"Virtual Side Doorbell:\n");
	ndev_dump_config_space(ndev,
			       &virt_reg[PLXNTB_VIRTUAL_DOORBELL_IRQ_SET_REG],
			       1, buf, &off, buf_size);

	off += scnprintf(buf + off, buf_size - off,
			"Virtual Side RID Table:\n");
	ndev_dump_rid_table(ndev, PLXNTB_VIRTUAL_SIDE, buf, &off, buf_size);

	off += scnprintf(buf + off, buf_size - off,
			"Link Side RID Table:\n");
	ndev_dump_rid_table(ndev, PLXNTB_LINK_SIDE, buf, &off, buf_size);

	ret = simple_read_from_buffer(ubuf, count, offp, buf, off);
	kfree(buf);

	return ret;
}

static ssize_t ndev_debugfs_info_read(struct file *filp, char __user *ubuf,
				      size_t count, loff_t *offp)
{
	struct plx_ntb_dev *ndev;
	char *buf;
	size_t buf_size;
	ssize_t ret, off;
	union { u64 v64; u32 v32; } u;
	u32 __iomem *virt_reg, __iomem *link_reg;

	ndev = filp->private_data;

	buf_size = min(count, 0x800UL);

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	virt_reg = ndev->local_mmio.virtual_pcireg;
	link_reg = ndev->local_mmio.link_pcireg;

	off = 0;

	/* Info device config */
	off += scnprintf(buf + off, buf_size - off,
			"NTB Device Information:\n");

	off += scnprintf(buf + off, buf_size - off,
			"\tUpstream Port\t\t\t#%d\n", ndev->ups_port);
	off += scnprintf(buf + off, buf_size - off,
			"\tNTB Port\t\t\t#%d\n", ndev->ntb_port);

	off += scnprintf(buf + off, buf_size - off,
			"\tNTB SW Local BAR\t\t#%d\n",
			ndev->switch_cfg->local_bar_num);

	off += scnprintf(buf + off, buf_size - off,
			"\tNTB SW Remote BAR\t\t#%d\n",
			ndev->switch_cfg->remote_bar_num);

	off += scnprintf(buf + off, buf_size - off,
			"\tNTB SW Memory Window[%d] BAR\t#%d\n",
			0, ndev->switch_cfg->mw_bar[0]);

	off += scnprintf(buf + off, buf_size - off,
			"\tNTB SW Memory Window[%d] BAR\t#%d\n",
			1, ndev->switch_cfg->mw_bar[1]);

	off += scnprintf(buf + off, buf_size - off,
			"\tNTB SW Link State:\t\tSpeed:Gen%d, Width:x%d\n",
			PLXNTB_LINK_SPEED(ndev->link_speed_width),
			PLXNTB_LINK_WIDTH(ndev->link_speed_width));

	off += scnprintf(buf + off, buf_size - off,
			"\tNTB SW Port State:\t\t");
	switch (ndev->port_state) {
	case NTB_PORT_DISABLED:
		off += scnprintf(buf + off, buf_size - off,
				 "Disabled\n");
		break;
	case NTB_PORT_ENABLED:
		off += scnprintf(buf + off, buf_size - off,
				 "Enabled\n");
		break;
	}

	/* Virtual side address */
	off += scnprintf(buf + off, buf_size - off,
			"Virtual Side - Address Map:\n");
	off += scnprintf(buf + off, buf_size - off,
			"\tBAR0:\tbase_address:\t\t0x%08x\n"
			"\t     \tsize:\t\t\t0x%08x\n"
			"\tBAR2:\tbase_address:\t\t0x%08x\n"
			"\t     \tsize:\t\t\t0x%08x\n"
			"\tBAR3:\tbase_address:\t\t0x%08x\n"
			"\t     \tsize:\t\t\t0x%08x\n"
			"\tBAR4:\tbase_address:\t\t0x%016llx\n"
			"\t     \tsize:\t\t\t0x%016llx\n"
			"\tMMIO:\tlocal_base_reg:\t\t0x%p\n"
			"\t     \tlocal_virt_reg:\t\t0x%p\n"
			"\t     \tlocal_link_reg:\t\t0x%p\n"
			"\t     \tlocal_rid_reg:\t\t0x%p\n"
			"\t     \tremote_base_reg:\t0x%p\n"
			"\t     \tremote_virt_reg:\t0x%p\n"
			"\t     \tremote_link_reg:\t0x%p\n",
			ndev->bar0_res.base,
			ndev->bar0_res.size,
			ndev->bar2_res.base,
			ndev->bar2_res.size,
			ndev->bar3_res.base,
			ndev->bar3_res.size,
			ndev->bar4_res.base64,
			ndev->bar4_res.size64,
			ndev->local_mmio.base_pcireg,
			ndev->local_mmio.virtual_pcireg,
			ndev->local_mmio.link_pcireg,
			ndev->local_mmio.virtual_rid_lookup_table,
			ndev->remote_mmio.base_pcireg,
			ndev->remote_mmio.virtual_pcireg,
			ndev->remote_mmio.link_pcireg);

	u.v32 = ioread32(&virt_reg[PLXNTB_BAR2_ADDRESS_TRANSLATION_LOWER_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tXLAT:\tBAR2 address:\t\t0x%08x\n", u.v32);

	u.v32 = ioread32(&virt_reg[PLXNTB_BAR3_ADDRESS_TRANSLATION_UPPER_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\t     \tBAR3 address:\t\t0x%08x\n", u.v32);

	u.v64 = ioread32(&virt_reg[PLXNTB_BAR5_ADDRESS_TRANSLATION_UPPER_REG]);
	u.v64 = (u.v64 << 32) |
		ioread32(&virt_reg[PLXNTB_BAR4_ADDRESS_TRANSLATION_LOWER_REG]);

	off += scnprintf(buf + off, buf_size - off,
			"\t     \tBAR4 address:\t\t0x%016llx\n", u.v64);

	/* Link side address */
	off += scnprintf(buf + off, buf_size - off,
			"Link Side - Address Map:\n");

	u.v32 = ioread32(&link_reg[PLXNTB_BAR0_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tBAR0:\tbase_address:\t\t0x%08x\n", u.v32);

	u.v32 = ioread32(&link_reg[PLXNTB_BAR2_REG]);
	u.v32 &= GENMASK(32, 4);
	off += scnprintf(buf + off, buf_size - off,
			"\tBAR2:\tbase_address:\t\t0x%08x\n", u.v32);

	u.v64 = ioread32(&link_reg[PLXNTB_BAR5_REG]);
	u.v64 = (u.v64 << 32) | ioread32(&link_reg[PLXNTB_BAR4_REG]);
	u.v64 &= GENMASK(63, 4);

	off += scnprintf(buf + off, buf_size - off,
			"\tBAR4:\tbase_address:\t\t0x%016llx\n", u.v64);

	off += scnprintf(buf + off, buf_size - off,
			"\tMMIO:\tbase_address:\t\t0x%p\n"
			"\t     \tlink_address:\t\t0x%p\n"
			"\t     \trid_address:\t\t0x%p\n",
			ndev->local_mmio.base_pcireg,
			ndev->local_mmio.link_pcireg,
			ndev->local_mmio.link_rid_lookup_table);

	u.v32 = ioread32(&link_reg[PLXNTB_BAR2_ADDRESS_TRANSLATION_LOWER_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\tXLAT:\tBAR2 address:\t\t0x%08x\n", u.v32);

	u.v32 = ioread32(&link_reg[PLXNTB_LINK_BAR2_SETUP_REG]);
	off += scnprintf(buf + off, buf_size - off,
			"\t     \tBAR2 setup:\t\t0x%08x\n", u.v32);

	u.v64 = ioread32(&link_reg[PLXNTB_BAR5_ADDRESS_TRANSLATION_UPPER_REG]);
	u.v64 = (u.v64 << 32) |
		ioread32(&link_reg[PLXNTB_BAR4_ADDRESS_TRANSLATION_LOWER_REG]);

	off += scnprintf(buf + off, buf_size - off,
			"\t     \tBAR4 address:\t\t0x%016llx\n", u.v64);

	u.v64 = ioread32(&link_reg[PLXNTB_LINK_BAR5_SETUP_REG]);
	u.v64 = (u.v64 << 32) |
		ioread32(&link_reg[PLXNTB_LINK_BAR4_SETUP_REG]);

	off += scnprintf(buf + off, buf_size - off,
			"\t     \tBAR4 setup:\t\t0x%016llx\n", u.v64);

	ret = simple_read_from_buffer(ubuf, count, offp, buf, off);
	kfree(buf);

	return ret;
}

static const struct file_operations plxntb_debugfs_info = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ndev_debugfs_info_read,
};

static const struct file_operations plxntb_debugfs_debug = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ndev_debugfs_debug_read,
};

static void plxntb_deinit_debugfs(struct plx_ntb_dev *ndev)
{
	debugfs_remove_recursive(ndev->debugfs_dir);
}

static void plxntb_init_debugfs(struct plx_ntb_dev *ndev)
{
	if (!debugfs_dir) {
		ndev->debugfs_dir = NULL;
		ndev->debugfs_info = NULL;
		ndev->debugfs_debug = NULL;
	} else {
		ndev->debugfs_dir =
			debugfs_create_dir(pci_name(ndev->ntb.pdev),
					   debugfs_dir);
		if (!ndev->debugfs_dir) {
			ndev->debugfs_info = NULL;
			ndev->debugfs_debug = NULL;

		} else {
			ndev->debugfs_info =
			debugfs_create_file("info", 00400,
					    ndev->debugfs_dir, ndev,
					    &plxntb_debugfs_info);

			ndev->debugfs_debug =
			debugfs_create_file("debug", 00400,
					    ndev->debugfs_dir, ndev,
					    &plxntb_debugfs_debug);
		}
	}
}

static void ndev_enable_interrupts(struct plx_ntb_dev *ndev)
{
	u32 __iomem *virt_reg, data32;
	struct pci_dev *pdev;

	virt_reg = ndev->local_mmio.virtual_pcireg;
	pdev = ndev->ntb.pdev;

	/* Enable DL active change of Link side being reported to Virtual Side
	 * Disable correctable, uncorrectable error, message drop reporting
	 */
	iowrite32(((PLXNTB_LINK_ERROR_MASK_CORRECTABLE_BIT |
		    PLXNTB_LINK_ERROR_MASK_UNCORRECTABLE_BIT |
		    PLXNTB_LINK_ERROR_MASK_MESSAGE_DROP_BIT) &
		   ~PLXNTB_LINK_ERROR_MASK_DL_ACTIVE_CHANGE_BIT),
		   &virt_reg[PLXNTB_LINK_ERROR_MASK_VIRTUAL_REG]);

	/* unmask doorbell interrupts on Virtual side */
	data32 = ioread32(&virt_reg[PLXNTB_VIRTUAL_DOORBELL_IRQ_MASK_SET_REG]);
	iowrite32(data32,
		  &virt_reg[PLXNTB_VIRTUAL_DOORBELL_IRQ_MASK_CLEAR_REG]);

	/* Verify one vector allocated by default */
	data32 = ioread32(&virt_reg[PLXNTB_MSI_CONTROL_REG]);
	data32 &= GENMASK(PLXNTB_MSI_MULTIPLE_MESSAGE_ENABLE_BIT + 2,
			  PLXNTB_MSI_MULTIPLE_MESSAGE_ENABLE_BIT);
	if (data32)
		dev_warn(&pdev->dev, "Multiple MSI vector allocated\n");

	/* unmask MSI interrupts */
	iowrite32(0, &virt_reg[PLXNTB_MSI_MASK_REG]);
}

static inline void plxntb_doorbell_event(struct plx_ntb_dev *ndev, int vec)
{
	u64 vec_mask;

	dev_dbg(&ndev->ntb.pdev->dev, "vec %d\n", vec);

	vec_mask = ndev_vec_mask(ndev, vec);

	/* if vector valid, callback to ntb api */
	if (vec_mask & ndev->db_valid_mask)
		ntb_db_event(&ndev->ntb, vec);
}

static inline void plxntb_link_event(struct plx_ntb_dev *ndev)
{
	u32 __iomem *virt_reg, link_change;

	virt_reg = ndev->local_mmio.virtual_pcireg;

	link_change = ioread32(&virt_reg[PLXNTB_LINK_ERROR_STATUS_VIRTUAL_REG]);

	/* if link status changed, callback to ntb api */
	if (link_change) {
		iowrite32(link_change,
			  &virt_reg[PLXNTB_LINK_ERROR_STATUS_VIRTUAL_REG]);

		if (ndev_is_link_state_changed(ndev))
			ntb_link_event(&ndev->ntb);
	}
}

static irqreturn_t ndev_irq_isr(int irq, void *dev)
{
	struct plx_ntb_dev *ndev = dev;

	plxntb_link_event(ndev);

	plxntb_doorbell_event(ndev, irq - ndev->ntb.pdev->irq);

	return IRQ_HANDLED;
}

static void ndev_deinit_isr(struct plx_ntb_dev *ndev)
{
	struct pci_dev *pdev;

	pdev = ndev->ntb.pdev;

	free_irq(pdev->irq, ndev);

	if (pci_dev_msi_enabled(pdev))
		pci_disable_msi(pdev);
}

static int ndev_init_isr(struct plx_ntb_dev *ndev)
{
	int rc;
	struct pci_dev *pdev;

	pdev = ndev->ntb.pdev;

	rc = pci_enable_msi(pdev);
	if (rc)
		goto err_msi_enable;

	/* one MSI vector no flag setting */
	rc = request_irq(pdev->irq, ndev_irq_isr, 0, "ndev_irq_isr", ndev);
	if (rc)
		goto err_msi_request;

	ndev->db_vec_count = 1;

	dev_dbg(&pdev->dev, "Using MSI interrupts\n");

	return 0;

err_msi_request:
	pci_disable_msi(pdev);
err_msi_enable:
	return rc;
}

static int ndev_insert_rid_entry(struct plx_ntb_dev *ndev,
				 enum ntb_rid_side ntb_side,
				 u8 bus, u8 slot, u8 func)
{
	u32 __iomem *rid_table, rid_value;
	u16 rid_entry;
	int rc = -EINVAL, i;

	if (ntb_side == PLXNTB_VIRTUAL_SIDE)
		rid_table = ndev->local_mmio.virtual_rid_lookup_table;
	else
		rid_table = ndev->local_mmio.link_rid_lookup_table;

	/* generate RID entry */
	rid_entry = (bus << PLXNTB_RID_BUS_SHIFT |
		     (slot << PLXNTB_RID_SLOT_SHIFT & PLXNTB_RID_SLOT_MASK) |
		     PLXNTB_RID_ENTRY_ENABLE);

	/* find if rid entry already exists in the table */
	for (i = 0; i < PLXNTB_RID_ENTRY_PAIRS; i++) {
		rid_value = ioread32(&rid_table[i]);

		if (rid_entry == (u16)rid_value ||
		    (rid_entry == (u16)(rid_value >> PLXNTB_RID_ENTRY_SHIFT))) {
			dev_dbg(&ndev->ntb.pdev->dev,
				"RID entry (0x%04x) exists\n", rid_entry);

			/* not return insert rid error, found the rid entry */
			return 0;
		}
	}

	/* insert the rid entry from lower first then upper entry */
	for (i = 0; i < PLXNTB_RID_ENTRY_PAIRS; i++) {
		rid_value = ioread32(&rid_table[i]);

		if (PLXNTB_RID_ENTRY_ENABLE !=
				(rid_value & PLXNTB_RID_ENTRY_ENABLE)) {
			rid_value &= PLXNTB_RID_ENTRY_MASK_LOWER;
			rid_value |= rid_entry;

			iowrite32(rid_value, &rid_table[i]);

			return 0;

		} else if (PLXNTB_RID_ENTRY_ENABLE_UPPER !=
			(rid_value & PLXNTB_RID_ENTRY_ENABLE_UPPER)) {
			rid_value &= PLXNTB_RID_ENTRY_MASK_UPPER;
			rid_value |= (rid_entry << PLXNTB_RID_ENTRY_SHIFT);

			iowrite32(rid_value, &rid_table[i]);

			return 0;
		}
	}

	return rc;
}

/* Intel IOAT Device ID supported for RID table */
static const u16 ioat_did_tbl[] = {
	PCI_DEVICE_ID_INTEL_IOAT_BDX0,
	PCI_DEVICE_ID_INTEL_IOAT_BDX1,
	PCI_DEVICE_ID_INTEL_IOAT_BDX2,
	PCI_DEVICE_ID_INTEL_IOAT_BDX3,
	PCI_DEVICE_ID_INTEL_IOAT_BDX4,
	PCI_DEVICE_ID_INTEL_IOAT_BDX5,
	PCI_DEVICE_ID_INTEL_IOAT_BDX6,
	PCI_DEVICE_ID_INTEL_IOAT_BDX7,
	PCI_DEVICE_ID_INTEL_IOAT_BDX8,
	PCI_DEVICE_ID_INTEL_IOAT_BDX9,

	PCI_DEVICE_ID_INTEL_IOAT_IVB0,
	PCI_DEVICE_ID_INTEL_IOAT_IVB1,
	PCI_DEVICE_ID_INTEL_IOAT_IVB2,
	PCI_DEVICE_ID_INTEL_IOAT_IVB3,
	PCI_DEVICE_ID_INTEL_IOAT_IVB4,
	PCI_DEVICE_ID_INTEL_IOAT_IVB5,
	PCI_DEVICE_ID_INTEL_IOAT_IVB6,
	PCI_DEVICE_ID_INTEL_IOAT_IVB7,
	PCI_DEVICE_ID_INTEL_IOAT_IVB8,
	PCI_DEVICE_ID_INTEL_IOAT_IVB9,
	0,
};

static void plxntb_init_virtual_rid(struct plx_ntb_dev *ndev)
{
	u8 bus, slot, func = 0;
	u32 __iomem *virt_reg, readback;
	struct pci_dev *ioat_pdev;
	int i;

	virt_reg = ndev->local_mmio.virtual_pcireg;

	for (i = 0; ioat_did_tbl[i] != 0; i++) {
		while ((ioat_pdev = pci_get_device(PCI_VENDOR_ID_INTEL,
						   ioat_did_tbl[i],
						   ioat_pdev)) != NULL) {
			bus = ioat_pdev->bus->number;
			slot = (u8)PCI_SLOT(ioat_pdev->devfn);

			ndev_insert_rid_entry(ndev, PLXNTB_VIRTUAL_SIDE,
					      bus, slot, func);
		}
	}

	readback = ioread32(&virt_reg[PLXNTB_REQUESTER_ID_READ_BACK]);

	/* root port connected to switch/NTB */
	bus = (u8)PCI_BUS_NUM(readback);
	slot = (u8)PCI_SLOT(readback);

	if (ndev_insert_rid_entry(ndev, PLXNTB_VIRTUAL_SIDE, bus, slot, func)) {
		dev_err(&ndev->ntb.pdev->dev,
			"RID entry %x:%x.%x insert failed\n",
			bus, slot, func);
	}

	/* memory access */
	slot = 0;
	if (ndev_insert_rid_entry(ndev, PLXNTB_VIRTUAL_SIDE, bus, slot, func)) {
		dev_err(&ndev->ntb.pdev->dev,
			"RID entry %x:%x.%x insert failed\n",
			bus, slot, func);
	}
}

static void plxntb_init_link_rid(struct plx_ntb_dev *ndev)
{
	u8 bus = 0, slot, func = 0;
	int rc;

	for (slot = 0; slot < 32; slot++) {
		rc = ndev_insert_rid_entry(ndev, PLXNTB_LINK_SIDE,
					   bus, slot, func);
		if (rc) {
			dev_err(&ndev->ntb.pdev->dev,
				"RID entry %x:%x.%x insert failed\n",
				bus, slot, func);
		}
	}
}

static void ndev_set_link_side_mw0(struct plx_ntb_dev *ndev)
{
	u32 __iomem *link_reg;
	u64 mask64;

	link_reg = ndev->local_mmio.link_pcireg;

	/* Set 64-bit memory window based on Virtual side's window size */
	mask64 = ndev->bar4_res.size64 - 1;
	mask64 = ~mask64;
	mask64 |= PLXNTB_BAR_64_BIT;
	mask64 |= PLXNTB_BAR_PREFETCHABLE;

	iowrite32((u32)mask64,
		  &link_reg[PLXNTB_LINK_BAR4_SETUP_REG]);

	iowrite32((u32)(mask64 >> 32),
		  &link_reg[PLXNTB_LINK_BAR5_SETUP_REG]);

	/* Set hardcoded address to BAR4/BAR5 */
	iowrite32(PLXNTB_BAR_MASK,
		  &link_reg[PLXNTB_BAR4_REG]);

	iowrite32(ndev->link_bar4,
		  &link_reg[PLXNTB_BAR4_REG]);

	iowrite32(PLXNTB_BAR_MASK,
		  &link_reg[PLXNTB_BAR5_REG]);

	iowrite32(ndev->link_bar5,
		  &link_reg[PLXNTB_BAR5_REG]);

	/* Set BAR4/BAR5 translation register to 0 for whole memory access */
	iowrite32(0,
		  &link_reg[PLXNTB_BAR4_ADDRESS_TRANSLATION_LOWER_REG]);

	iowrite32(0,
		  &link_reg[PLXNTB_BAR5_ADDRESS_TRANSLATION_UPPER_REG]);
}

static void ndev_set_link_side_mw1(struct plx_ntb_dev *ndev)
{
	u32 __iomem *link_reg;
	u32 mask32;

	link_reg = ndev->local_mmio.link_pcireg;

	/* Set 32-bit memory window based on Virtual side's window size */
	mask32 = ndev->bar2_res.size - 1;
	mask32 = ~mask32;
	mask32 |= PLXNTB_BAR_PREFETCHABLE;

	iowrite32((u32)mask32,
		  &link_reg[PLXNTB_LINK_BAR2_SETUP_REG]);

	/* Set hardcoded address to BAR2 */
	iowrite32(PLXNTB_BAR_MASK,
		  &link_reg[PLXNTB_BAR2_REG]);

	iowrite32(ndev->link_bar2,
		  &link_reg[PLXNTB_BAR2_REG]);

	/* Set BAR2 translation register to 0 for whole memory access */
	iowrite32(0,
		  &link_reg[PLXNTB_BAR2_ADDRESS_TRANSLATION_LOWER_REG]);
}

static void plxntb_enumerate_link_side(struct plx_ntb_dev *ndev)
{
	u32 __iomem *link_reg, data32;
	const int ERROR_STATUS_SHIFT = 16;

	link_reg = ndev->local_mmio.link_pcireg;

	/* Set hardcoded address to BAR0 */
	iowrite32(PLXNTB_BAR_MASK,
		  &link_reg[PLXNTB_BAR0_REG]);

	iowrite32(ndev->link_bar0,
		  &link_reg[PLXNTB_BAR0_REG]);

	/* Set BAR and Setup register according to memory window */
	if (ndev->mw_count == 2) {
		ndev_set_link_side_mw0(ndev);
		ndev_set_link_side_mw1(ndev);
	} else {
		ndev_set_link_side_mw0(ndev);
	}

	/* Enable Memory Access, Bus Master */
	iowrite32or(PLXNTB_COMMAND_MEMORY_ACCESS_ENABLE |
		    PLXNTB_COMMAND_BUS_MASTER_ENABLE,
		    &link_reg[PLXNTB_COMMAND_REG]);

	/* Disable SERR */
	iowrite32and((u32)~PLXNTB_COMMAND_SERR_ENABLE,
		     &link_reg[PLXNTB_COMMAND_REG]);

	/* Set Maximum Payload Size of 256-byte, No Snoop
	 * Disable error reporting and clear device status
	 */
	data32 = (PLXNTB_DEVSTS_AND_CONTROL_MAX_PAYLOAD_256 |
		  PLXNTB_DEVSTS_AND_CONTROL_NO_SNOOP_ENABLE);

	data32 |= (0xFF << ERROR_STATUS_SHIFT);

	iowrite32(data32,
		  &link_reg[PLXNTB_DEVSTS_AND_CONTROL_REG]);

	/* Enable capability to link retrain and disable via Control register */
	iowrite32or(PLXNTB_PM_HOT_PLUG_CONTROL_WRITE_ENABEL,
		    &link_reg[PLXNTB_PM_HOT_PLUG_REG]);

	/* Set drop poisoned TLP */
	iowrite32or(PLXNTB_INGRESS_CONTROL_DROP_POISONED_TLPS,
		    &link_reg[PLXNTB_INGRESS_CONTROL_REG]);

	/* Set all uncorrectable errors to fatal */
	iowrite32or(PLXNTB_UNCORRECTABLE_ERROR_ALL,
		    &link_reg[PLXNTB_UNCORRECTABLE_ERROR_SEVERITY_REG]);

	/* Clear correctable error register */
	data32 = ioread32(&link_reg[PLXNTB_CORRECTABLE_ERROR_STATUS_REG]);
	if (data32) {
		iowrite32(data32,
			  &link_reg[PLXNTB_CORRECTABLE_ERROR_STATUS_REG]);
	}

	/* Initialize RID table on Link side */
	plxntb_init_link_rid(ndev);
}

static void plxntb_enumerate_virtual_side(struct plx_ntb_dev *ndev)
{
	u32 __iomem *virt_reg, data32;

	virt_reg = ndev->local_mmio.virtual_pcireg;

	/* Disable interrupt in PCI Command register as MSI requires */
	iowrite32or(PLXNTB_COMMAND_INTERRUPT_DISABLE,
		    &virt_reg[PLXNTB_COMMAND_REG]);

	/* Disable SERR */
	iowrite32and((u32)~PLXNTB_COMMAND_SERR_ENABLE,
		     &virt_reg[PLXNTB_COMMAND_REG]);

	/* Set BAR2 translation address to Link side's BAR0 */
	iowrite32(ndev->link_bar2,
		  &virt_reg[PLXNTB_BAR2_ADDRESS_TRANSLATION_LOWER_REG]);

	/* Set BAR3 translation address to Link side's BAR0 */
	iowrite32(ndev->link_bar0,
		  &virt_reg[PLXNTB_BAR3_ADDRESS_TRANSLATION_UPPER_REG]);

	/* Set BAR4/BAR5 translation address to Link side's BAR2/BAR3 */
	iowrite32(ndev->link_bar4,
		  &virt_reg[PLXNTB_BAR4_ADDRESS_TRANSLATION_LOWER_REG]);

	iowrite32(ndev->link_bar5,
		  &virt_reg[PLXNTB_BAR5_ADDRESS_TRANSLATION_UPPER_REG]);

	/* Clear correctable error register */
	data32 = ioread32(&virt_reg[PLXNTB_CORRECTABLE_ERROR_STATUS_REG]);
	if (data32) {
		iowrite32(data32,
			  &virt_reg[PLXNTB_CORRECTABLE_ERROR_STATUS_REG]);
	}

	/* Initialize RID table on Virtual side */
	plxntb_init_virtual_rid(ndev);

	/* Clear Link error status as reflected by the Virtual side */
	data32 = ioread32(&virt_reg[PLXNTB_LINK_ERROR_STATUS_VIRTUAL_REG]);
	if (data32) {
		iowrite32(data32,
			  &virt_reg[PLXNTB_LINK_ERROR_STATUS_VIRTUAL_REG]);
	}
}

static void ndev_apply_link_hack(struct plx_ntb_dev *ndev)
{
	u32 __iomem *base_reg, port_basereg;

	base_reg = ndev->local_mmio.base_pcireg;

	/* upstream port */
	port_basereg = ndev_get_port_base_reg(ndev->ups_port);
	base_reg = &base_reg[port_basereg];

	/* Disable reset to Link side once DL_Down
	 * Enable access to NT0 virtual interface and NT0 link interface
	 */
	iowrite32((PLXNTB_VS_DEBUG_DL_DOWN_RESET_DISABLE_BIT |
		   PLXNTB_VS_DEBUG_NT0_VIRTUAL_ACCESS_ENABLE_BIT |
		   PLXNTB_VS_DEBUG_NT0_LINK_ACCESS_ENABLE_BIT),
		  &base_reg[PLXNTB_VS_DEBUG_REG]);
}

static void ndev_disable_correctable_error_reporting(struct plx_ntb_dev *ndev)
{
	u32 __iomem *base_reg, port_basereg;

	base_reg = ndev->local_mmio.base_pcireg;

	/* upstream port, mask all correctable error reporting */
	port_basereg = ndev_get_port_base_reg(ndev->ups_port);
	base_reg = &base_reg[port_basereg];

	iowrite32or(PLXNTB_CORRECTABLE_ERROR_ALL,
		    &base_reg[PLXNTB_CORRECTABLE_ERROR_MASK_REG]);

	/* ntb port, mask all correctable error reporting */
	port_basereg = ndev_get_port_base_reg(ndev->ntb_port);
	base_reg = &base_reg[port_basereg];

	iowrite32or(PLXNTB_CORRECTABLE_ERROR_ALL,
		    &base_reg[PLXNTB_CORRECTABLE_ERROR_MASK_REG]);
}

static enum switch_port_num ndev_get_switch_port(struct plx_ntb_dev *ndev,
						 enum switch_port sw_port)
{
	u32 __iomem *base_reg, vs0_ups;
	enum switch_port_num port;

	base_reg = ndev->local_mmio.base_pcireg;

	vs0_ups = ioread32(&base_reg[PLXNTB_VS0_UPSTREAM_REG]);

	if (sw_port == UPSTREAM_PORT)
		port = vs0_ups & PLXNTB_VS0_UPSTREAM_PORT_MASK;

	else if (sw_port == NTB_PORT)
		port = (vs0_ups >> PLXNTB_VS0_UPSTREAM_NT0_PORT_SHIFT) &
			  PLXNTB_VS0_UPSTREAM_PORT_MASK;

	return port;
}

static void ndev_get_bar_resources(struct plx_ntb_dev *ndev, int bar)
{
	struct pci_dev *pdev;
	struct ntb_bar_resources *bar_res;

	pdev = ndev->ntb.pdev;

	switch (bar) {
	case NTB_BAR_NUM_ZERO:
		bar_res = &ndev->bar0_res;
		break;
	case NTB_BAR_NUM_TWO:
		bar_res = &ndev->bar2_res;
		break;
	case NTB_BAR_NUM_THREE:
		bar_res = &ndev->bar3_res;
		break;
	case NTB_BAR_NUM_FOUR:
		bar_res = &ndev->bar4_res;
		break;
	}

	bar_res->type = pci_resource_flags(pdev, bar);

	if (bar_res->type & IORESOURCE_MEM_64) {
		bar_res->base64 = pci_resource_start(pdev, bar);
		bar_res->size64 = pci_resource_len(pdev, bar);
		bar_res->base = 0;
		bar_res->size = 0;
	} else {
		bar_res->base = pci_resource_start(pdev, bar);
		bar_res->size = pci_resource_len(pdev, bar);
		bar_res->base64 = 0;
		bar_res->size64 = 0;
	}
}

static void plxntb_init_struct(struct plx_ntb_dev *ndev)
{
	u32 __iomem *base_reg;

	ndev->mw_count = PLXNTB_MW_COUNT;
	ndev->db_count = PLXNTB_DB_COUNT;
	ndev->spad_count = PLXNTB_SPAD_COUNT;

	ndev->db_valid_mask = BIT_ULL(ndev->db_count) - 1;
	ndev->db_vec_shift = PLXNTB_DB_SHIFT;

	ndev_reset_unsafe_flags(ndev);

	/* Get BARs resources */
	ndev_get_bar_resources(ndev, NTB_BAR_NUM_ZERO);
	ndev_get_bar_resources(ndev, NTB_BAR_NUM_TWO);
	ndev_get_bar_resources(ndev, NTB_BAR_NUM_THREE);
	ndev_get_bar_resources(ndev, NTB_BAR_NUM_FOUR);

	/* memory window 0 */
	dev_dbg(&ndev->ntb.pdev->dev, "BAR2 base:0x%08x, size:0x%08x\n",
		ndev->bar2_res.base, ndev->bar2_res.size);

	/* memory window 1 */
	dev_dbg(&ndev->ntb.pdev->dev, "BAR4 base:0x%016llx, size:0x%016llx\n",
		ndev->bar4_res.base64, ndev->bar4_res.size64);

	/* MMIO housekeeper for local-side */
	base_reg = ndev->local_mmio.base_pcireg;

	ndev->local_mmio.virtual_pcireg =
		&base_reg[PLXNTB_NT0_VIRTUAL_BASE_REG];

	ndev->local_mmio.link_pcireg =
		&base_reg[PLXNTB_NT0_LINK_BASE_REG];

	ndev->local_mmio.virtual_rid_lookup_table =
		&ndev->local_mmio.virtual_pcireg[PLXNTB_RID_VIRTUAL_ARRAY_REG];

	ndev->local_mmio.link_rid_lookup_table =
		&ndev->local_mmio.link_pcireg[PLXNTB_RID_LINK_ARRAY_REG];

	/* Set hardcoded BARs to Link side as translation address target */
	ndev->link_bar0 = PLXNTB_NTB0_LINK_BAR0;
	ndev->link_bar2 = PLXNTB_NTB0_LINK_BAR2;
	ndev->link_bar4 = PLXNTB_NTB0_LINK_BAR4;
	ndev->link_bar5 = PLXNTB_NTB0_LINK_BAR5;

	/* MMIO housekeeper for remote-side */
	base_reg = ndev->remote_mmio.base_pcireg;

	ndev->remote_mmio.virtual_pcireg =
		&base_reg[PLXNTB_NT0_VIRTUAL_BASE_REG];

	ndev->remote_mmio.link_pcireg =
		&base_reg[PLXNTB_NT0_LINK_BASE_REG];

	/* Get Upstream and NTB port number */
	ndev->ups_port = ndev_get_switch_port(ndev, UPSTREAM_PORT);

	ndev->ntb_port = ndev_get_switch_port(ndev, NTB_PORT);
}

static void plxntb_deinit_dev(struct plx_ntb_dev *ndev)
{
	ndev_deinit_isr(ndev);
}

static int plxntb_init_dev(struct plx_ntb_dev *ndev)
{
	int rc;

	plxntb_init_struct(ndev);

	ndev_disable_correctable_error_reporting(ndev);

	ndev_apply_link_hack(ndev);

	/* as back-to-back topology, no root complex enumerates on Link side */
	plxntb_enumerate_link_side(ndev);

	/* ntb settings in addition to firmware enumeration on Virtual side */
	plxntb_enumerate_virtual_side(ndev);

	ndev_enable_interrupts(ndev);

	rc = ndev_init_isr(ndev);
	if (rc)
		dev_err(&ndev->ntb.pdev->dev, "ISR init failed\n");

	return rc;
}

/* This driver supports plx switch config with:
 * One NTB port, as NTB0
 * Address Lookup Table, and PLX DMA are disabled
 *
 * Device register access:
 * BAR0, local device access and control
 * BAR3, remote device access
 *
 * Remote memory window access (set mw0 as 64-bit accessible):
 * BAR4, memory window 0 (mw0), 64-bit address
 * BAR2, memory window 1 (mw1), 32-bit address
 */
static const struct switch_config plxntb_switch_conifg = {
	.local_bar_num = NTB_BAR_NUM_ZERO,
	.remote_bar_num = NTB_BAR_NUM_THREE,
	.mw_bar = {NTB_BAR_NUM_FOUR, NTB_BAR_NUM_TWO},
	.mw_addr_size = {64, 32},
};

static int plxntb_check_switch_config(struct plx_ntb_dev *ndev)
{
	u32 __iomem *base_reg, vs0_ups;
	u32 supported_config, mask, bar_type;
	int i;

	base_reg = ndev->local_mmio.base_pcireg;
	vs0_ups = ioread32(&base_reg[PLXNTB_VS0_UPSTREAM_REG]);

	/* check NTB0 enabled, NTB1, PLX DMA and address LUT disabled */
	supported_config = PLXNTB_VS0_UPSTREAM_NT0_ENABLE &
			  (~PLXNTB_VS0_UPSTREAM_NT1_ENABLE) &
			  (~PLXNTB_VS0_UPSTREAM_DMA_MODE) &
			  (~PLXNTB_VS0_UPSTREAM_NT0_A_LUT_ENABLE) &
			  (~PLXNTB_VS0_UPSTREAM_NT1_A_LUT_ENABLE);

	/* build up mask of don't care bit */
	mask = GENMASK(12, 0) | GENMASK(20, 14) | GENMASK(29, 22);

	if ((vs0_ups | mask) != (supported_config | mask)) {
		dev_err(&ndev->ntb.pdev->dev,
			"Switch config check failed vs0(0x%08x)\n",
			vs0_ups);

		return -EINVAL;
	}

	/* check mw BAR setup */
	for (i = 0; i < PLXNTB_MW_COUNT; i++) {
		bar_type = pci_resource_flags(ndev->ntb.pdev,
					      ndev->switch_cfg->mw_bar[i]);

		if (ndev->switch_cfg->mw_addr_size[i] == 64) {
			if (!(bar_type & IORESOURCE_MEM_64)) {
				dev_err(&ndev->ntb.pdev->dev,
					"Switch check failed BAR%d\n",
					ndev->switch_cfg->mw_bar[i]);

				return -EINVAL;
			}

		} else if (ndev->switch_cfg->mw_addr_size[i] == 32) {
			if (bar_type & IORESOURCE_MEM_64) {
				dev_err(&ndev->ntb.pdev->dev,
					"Switch check failed BAR%d\n",
					ndev->switch_cfg->mw_bar[i]);

				return -EINVAL;
			}
		}
	}

	return 0;
}

static int plxntb_init_pci(struct plx_ntb_dev *ndev, struct pci_dev *pdev)
{
	int rc;

	pci_set_drvdata(pdev, ndev);

	rc = pci_enable_device(pdev);
	if (rc)
		goto err_pci_enable;

	rc = pci_request_regions(pdev, NTB_NAME);
	if (rc)
		goto err_pci_regions;

	pci_set_master(pdev);

	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if (rc) {
		rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
		if (rc)
			goto err_dma_mask;
		dev_warn(&pdev->dev, "Cannot DMA highmem\n");
	}

	rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
	if (rc) {
		rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
		if (rc)
			goto err_dma_mask;
		dev_warn(&pdev->dev, "Cannot DMA consistent highmem");
	}

	rc = dma_coerce_mask_and_coherent(&ndev->ntb.dev,
					  dma_get_mask(&pdev->dev));

	/* local mmio access */
	ndev->local_mmio.base_pcireg =
			pci_iomap(pdev, ndev->switch_cfg->local_bar_num, 0);

	if (!ndev->local_mmio.base_pcireg) {
		rc = -EIO;
		goto err_local_mmio;
	}

	/* remote mmio access */
	ndev->remote_mmio.base_pcireg =
			pci_iomap(pdev, ndev->switch_cfg->remote_bar_num, 0);

	if (!ndev->remote_mmio.base_pcireg) {
		rc = -EIO;
		goto err_remote_mmio;
	}

	return 0;

err_remote_mmio:
	pci_iounmap(pdev, ndev->local_mmio.base_pcireg);
err_local_mmio:
err_dma_mask:
	pci_clear_master(pdev);
	pci_release_regions(pdev);
err_pci_regions:
	pci_disable_device(pdev);
err_pci_enable:
	pci_set_drvdata(pdev, NULL);
	return rc;
}

static void plxntb_deinit_pci(struct plx_ntb_dev *ndev)
{
	struct pci_dev *pdev = ndev->ntb.pdev;

	pci_iounmap(pdev, ndev->remote_mmio.base_pcireg);
	pci_iounmap(pdev, ndev->local_mmio.base_pcireg);

	pci_clear_master(pdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

static void plxntb_init_early_struct(struct plx_ntb_dev *ndev,
				     struct pci_dev *pdev)
{
	ndev->ntb.pdev = pdev;
	ndev->ntb.topo = NTB_TOPO_SWITCH;
	ndev->ntb.ops = &plx_ntb_ops;

	ndev->local_mmio.virtual_rid_lookup_table = NULL;
	ndev->local_mmio.link_rid_lookup_table = NULL;
	ndev->remote_mmio.virtual_rid_lookup_table = NULL;
	ndev->remote_mmio.link_rid_lookup_table = NULL;

	ndev->switch_cfg = &plxntb_switch_conifg;

	ndev->port_state = NTB_PORT_DISABLED;
	ndev->ups_port = 0;
	ndev->ntb_port = 0;

	ndev->link_speed_width = 0;

	ndev->mw_count = 0;
	ndev->spad_count = 0;
	ndev->db_count = 0;
	ndev->db_vec_count = 0;
	ndev->db_vec_shift = 0;

	ndev->db_valid_mask = 0;
	ndev->db_link_mask = 0;
	ndev->db_mask = 0;

	spin_lock_init(&ndev->db_mask_lock);
}

static int plxntb_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	struct plx_ntb_dev *ndev;
	int rc, node;

	node = dev_to_node(&pdev->dev);

	ndev = kzalloc_node(sizeof(*ndev), GFP_KERNEL, node);
	if (!ndev) {
		rc = -ENOMEM;
		goto err_ndev;
	}

	plxntb_init_early_struct(ndev, pdev);

	rc = plxntb_init_pci(ndev, pdev);
	if (rc)
		goto err_init_pci;

	rc = plxntb_check_switch_config(ndev);
	if (rc)
		goto err_check_config;

	rc = plxntb_init_dev(ndev);
	if (rc)
		goto err_init_dev;

	plxntb_init_debugfs(ndev);

	rc = ntb_register_device(&ndev->ntb);
	if (rc)
		goto err_register;

	dev_info(&pdev->dev, "NTB device registered.\n");

	return 0;

err_register:
	plxntb_deinit_debugfs(ndev);
	plxntb_deinit_dev(ndev);
err_init_dev:
err_check_config:
	plxntb_deinit_pci(ndev);
err_init_pci:
	kfree(ndev);
err_ndev:
	return rc;
}

static void plxntb_pci_remove(struct pci_dev *pdev)
{
	struct plx_ntb_dev *ndev = pci_get_drvdata(pdev);

	ntb_unregister_device(&ndev->ntb);
	plxntb_deinit_debugfs(ndev);
	plxntb_deinit_dev(ndev);
	plxntb_deinit_pci(ndev);
	kfree(ndev);
}

static const struct pci_device_id plxntb_pci_tbl[] = {
	{PCI_VDEVICE(PLX, PCI_DEVICE_ID_PLX_NTB_NT0)},
	{0}
};
MODULE_DEVICE_TABLE(pci, plxntb_pci_tbl);

static struct pci_driver plxntb_pci_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= plxntb_pci_tbl,
	.probe		= plxntb_pci_probe,
	.remove		= plxntb_pci_remove,
};

static int __init plxntb_pci_driver_init(void)
{
	pr_info("%s %s\n", NTB_DESC, NTB_VER);

	if (debugfs_initialized())
		debugfs_dir = debugfs_create_dir(KBUILD_MODNAME, NULL);

	return pci_register_driver(&plxntb_pci_driver);
}
module_init(plxntb_pci_driver_init);

static void __exit plxntb_pci_driver_exit(void)
{
	pci_unregister_driver(&plxntb_pci_driver);

	debugfs_remove_recursive(debugfs_dir);
}
module_exit(plxntb_pci_driver_exit);
