/*
 * RISC-V translation routines for the RV64M Standard Extension.
 *
 * Copyright (c) 2016-2017 Sagar Karandikar, sagark@eecs.berkeley.edu
 * Copyright (c) 2018 Peer Adelt, peer.adelt@hni.uni-paderborn.de
 *                    Bastian Koppelmann, kbastian@mail.uni-paderborn.de
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */


static bool trans_aes32esmi(DisasContext *ctx, arg_aes32esmi *a)
{
	REQUIRE_EXT(ctx, RVK);
	TCGv bs;
	bs = tcg_const_tl(a->bs);
	gen_helper_aes32esmi(cpu_gpr[a->rs1], cpu_gpr[a->rs1], cpu_gpr[a->rs2], bs);
	return true;
}

static bool trans_aes32esi(DisasContext *ctx, arg_aes32esi *a)
{
	REQUIRE_EXT(ctx, RVK);
	TCGv bs;
	bs = tcg_const_tl(a->bs);
	gen_helper_aes32esi(cpu_gpr[a->rs1], cpu_gpr[a->rs1], cpu_gpr[a->rs2], bs);
	return true;
}

static bool trans_aes32dsmi(DisasContext *ctx, arg_aes32dsmi *a)
{
	REQUIRE_EXT(ctx, RVK);
	TCGv bs;
	bs = tcg_const_tl(a->bs);
	gen_helper_aes32dsmi(cpu_gpr[a->rs1], cpu_gpr[a->rs1], cpu_gpr[a->rs2], bs);
	return true;
}

static bool trans_aes32dsi(DisasContext *ctx, arg_aes32dsi *a)
{
	REQUIRE_EXT(ctx, RVK);
	TCGv bs;
	bs = tcg_const_tl(a->bs);
	gen_helper_aes32dsi(cpu_gpr[a->rs1], cpu_gpr[a->rs1], cpu_gpr[a->rs2], bs);
	return true;
}

#ifdef TARGET_RISCV64

#endif
