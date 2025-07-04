// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under both the BSD-style license (found in the
 * LICENSE file in the root directory of this source tree) and the GPLv2 (found
 * in the COPYING file in the root directory of this source tree).
 * You may select, at your option, one of the above-listed licenses.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/zstd.h>

#include "common/zstd_deps.h"
#include "common/zstd_internal.h"
#include "compress/zstd_compress_internal.h"

#define ZSTD_FORWARD_IF_ERR(ret)            \
	do {                                \
		size_t const __ret = (ret); \
		if (ZSTD_isError(__ret))    \
			return __ret;       \
	} while (0)

static size_t zstd_cctx_init(zstd_cctx *cctx, const zstd_parameters *parameters,
	unsigned long long pledged_src_size)
{
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_reset(
		cctx, ZSTD_reset_session_and_parameters));
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_setPledgedSrcSize(
		cctx, pledged_src_size));
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_setParameter(
		cctx, ZSTD_c_windowLog, parameters->cParams.windowLog));
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_setParameter(
		cctx, ZSTD_c_hashLog, parameters->cParams.hashLog));
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_setParameter(
		cctx, ZSTD_c_chainLog, parameters->cParams.chainLog));
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_setParameter(
		cctx, ZSTD_c_searchLog, parameters->cParams.searchLog));
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_setParameter(
		cctx, ZSTD_c_minMatch, parameters->cParams.minMatch));
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_setParameter(
		cctx, ZSTD_c_targetLength, parameters->cParams.targetLength));
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_setParameter(
		cctx, ZSTD_c_strategy, parameters->cParams.strategy));
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_setParameter(
		cctx, ZSTD_c_contentSizeFlag, parameters->fParams.contentSizeFlag));
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_setParameter(
		cctx, ZSTD_c_checksumFlag, parameters->fParams.checksumFlag));
	ZSTD_FORWARD_IF_ERR(ZSTD_CCtx_setParameter(
		cctx, ZSTD_c_dictIDFlag, !parameters->fParams.noDictIDFlag));
	return 0;
}

int zstd_min_clevel(void)
{
	return ZSTD_minCLevel();
}
EXPORT_SYMBOL(zstd_min_clevel);

int zstd_max_clevel(void)
{
	return ZSTD_maxCLevel();
}
EXPORT_SYMBOL(zstd_max_clevel);

int zstd_default_clevel(void)
{
	return ZSTD_defaultCLevel();
}
EXPORT_SYMBOL(zstd_default_clevel);

size_t zstd_compress_bound(size_t src_size)
{
	return ZSTD_compressBound(src_size);
}
EXPORT_SYMBOL(zstd_compress_bound);

zstd_parameters zstd_get_params(int level,
	unsigned long long estimated_src_size)
{
	return ZSTD_getParams(level, estimated_src_size, 0);
}
EXPORT_SYMBOL(zstd_get_params);

zstd_compression_parameters zstd_get_cparams(int level,
	unsigned long long estimated_src_size, size_t dict_size)
{
	return ZSTD_getCParams(level, estimated_src_size, dict_size);
}
EXPORT_SYMBOL(zstd_get_cparams);

size_t zstd_cctx_set_param(zstd_cctx *cctx, ZSTD_cParameter param, int value)
{
	return ZSTD_CCtx_setParameter(cctx, param, value);
}
EXPORT_SYMBOL(zstd_cctx_set_param);

size_t zstd_cctx_workspace_bound(const zstd_compression_parameters *cparams)
{
	return ZSTD_estimateCCtxSize_usingCParams(*cparams);
}
EXPORT_SYMBOL(zstd_cctx_workspace_bound);

// Used by zstd_cctx_workspace_bound_with_ext_seq_prod()
static size_t dummy_external_sequence_producer(
	void *sequenceProducerState,
	ZSTD_Sequence *outSeqs, size_t outSeqsCapacity,
	const void *src, size_t srcSize,
	const void *dict, size_t dictSize,
	int compressionLevel,
	size_t windowSize)
{
	(void)sequenceProducerState;
	(void)outSeqs; (void)outSeqsCapacity;
	(void)src; (void)srcSize;
	(void)dict; (void)dictSize;
	(void)compressionLevel;
	(void)windowSize;
	return ZSTD_SEQUENCE_PRODUCER_ERROR;
}

static void init_cctx_params_from_compress_params(
	ZSTD_CCtx_params *cctx_params,
	const zstd_compression_parameters *compress_params)
{
	ZSTD_parameters zstd_params;
	memset(&zstd_params, 0, sizeof(zstd_params));
	zstd_params.cParams = *compress_params;
	ZSTD_CCtxParams_init_advanced(cctx_params, zstd_params);
}

size_t zstd_cctx_workspace_bound_with_ext_seq_prod(const zstd_compression_parameters *compress_params)
{
	ZSTD_CCtx_params cctx_params;
	init_cctx_params_from_compress_params(&cctx_params, compress_params);
	ZSTD_CCtxParams_registerSequenceProducer(&cctx_params, NULL, dummy_external_sequence_producer);
	return ZSTD_estimateCCtxSize_usingCCtxParams(&cctx_params);
}
EXPORT_SYMBOL(zstd_cctx_workspace_bound_with_ext_seq_prod);

size_t zstd_cstream_workspace_bound_with_ext_seq_prod(const zstd_compression_parameters *compress_params)
{
	ZSTD_CCtx_params cctx_params;
	init_cctx_params_from_compress_params(&cctx_params, compress_params);
	ZSTD_CCtxParams_registerSequenceProducer(&cctx_params, NULL, dummy_external_sequence_producer);
	return ZSTD_estimateCStreamSize_usingCCtxParams(&cctx_params);
}
EXPORT_SYMBOL(zstd_cstream_workspace_bound_with_ext_seq_prod);

zstd_cctx *zstd_init_cctx(void *workspace, size_t workspace_size)
{
	if (workspace == NULL)
		return NULL;
	return ZSTD_initStaticCCtx(workspace, workspace_size);
}
EXPORT_SYMBOL(zstd_init_cctx);

zstd_cctx *zstd_create_cctx_advanced(zstd_custom_mem custom_mem)
{
	return ZSTD_createCCtx_advanced(custom_mem);
}
EXPORT_SYMBOL(zstd_create_cctx_advanced);

size_t zstd_free_cctx(zstd_cctx *cctx)
{
	return ZSTD_freeCCtx(cctx);
}
EXPORT_SYMBOL(zstd_free_cctx);

zstd_cdict *zstd_create_cdict_byreference(const void *dict, size_t dict_size,
					  zstd_compression_parameters cparams,
					  zstd_custom_mem custom_mem)
{
	return ZSTD_createCDict_advanced(dict, dict_size, ZSTD_dlm_byRef,
					 ZSTD_dct_auto, cparams, custom_mem);
}
EXPORT_SYMBOL(zstd_create_cdict_byreference);

size_t zstd_free_cdict(zstd_cdict *cdict)
{
	return ZSTD_freeCDict(cdict);
}
EXPORT_SYMBOL(zstd_free_cdict);

size_t zstd_compress_cctx(zstd_cctx *cctx, void *dst, size_t dst_capacity,
	const void *src, size_t src_size, const zstd_parameters *parameters)
{
	ZSTD_FORWARD_IF_ERR(zstd_cctx_init(cctx, parameters, src_size));
	return ZSTD_compress2(cctx, dst, dst_capacity, src, src_size);
}
EXPORT_SYMBOL(zstd_compress_cctx);

size_t zstd_compress_using_cdict(zstd_cctx *cctx, void *dst,
	size_t dst_capacity, const void *src, size_t src_size,
	const ZSTD_CDict *cdict)
{
	return ZSTD_compress_usingCDict(cctx, dst, dst_capacity,
					src, src_size, cdict);
}
EXPORT_SYMBOL(zstd_compress_using_cdict);

size_t zstd_cstream_workspace_bound(const zstd_compression_parameters *cparams)
{
	return ZSTD_estimateCStreamSize_usingCParams(*cparams);
}
EXPORT_SYMBOL(zstd_cstream_workspace_bound);

size_t ZSTD_CStreamWorkspaceBound(ZSTD_compressionParameters *cparams)
{
	return zstd_cstream_workspace_bound(cparams);
}
EXPORT_SYMBOL(ZSTD_CStreamWorkspaceBound);

zstd_cstream *zstd_init_cstream(const zstd_parameters *parameters,
	unsigned long long pledged_src_size, void *workspace, size_t workspace_size)
{
	zstd_cstream *cstream;

	if (workspace == NULL)
		return NULL;

	cstream = ZSTD_initStaticCStream(workspace, workspace_size);
	if (cstream == NULL)
		return NULL;

	/* 0 means unknown in linux zstd API but means 0 in new zstd API */
	if (pledged_src_size == 0)
		pledged_src_size = ZSTD_CONTENTSIZE_UNKNOWN;

	if (ZSTD_isError(zstd_cctx_init(cstream, parameters, pledged_src_size)))
		return NULL;

	return cstream;
}
EXPORT_SYMBOL(zstd_init_cstream);

size_t zstd_reset_cstream(zstd_cstream *cstream,
	unsigned long long pledged_src_size)
{
	if (pledged_src_size == 0)
		pledged_src_size = ZSTD_CONTENTSIZE_UNKNOWN;
	ZSTD_FORWARD_IF_ERR( ZSTD_CCtx_reset(cstream, ZSTD_reset_session_only) );
	ZSTD_FORWARD_IF_ERR( ZSTD_CCtx_setPledgedSrcSize(cstream, pledged_src_size) );
	return 0;
}
EXPORT_SYMBOL(zstd_reset_cstream);

size_t zstd_compress_stream(zstd_cstream *cstream, zstd_out_buffer *output,
	zstd_in_buffer *input)
{
	return ZSTD_compressStream(cstream, output, input);
}
EXPORT_SYMBOL(zstd_compress_stream);

size_t zstd_flush_stream(zstd_cstream *cstream, zstd_out_buffer *output)
{
	return ZSTD_flushStream(cstream, output);
}
EXPORT_SYMBOL(zstd_flush_stream);

size_t zstd_end_stream(zstd_cstream *cstream, zstd_out_buffer *output)
{
	return ZSTD_endStream(cstream, output);
}
EXPORT_SYMBOL(zstd_end_stream);

void zstd_register_sequence_producer(
  zstd_cctx *cctx,
  void* sequence_producer_state,
  zstd_sequence_producer_f sequence_producer
) {
	ZSTD_registerSequenceProducer(cctx, sequence_producer_state, sequence_producer);
}
EXPORT_SYMBOL(zstd_register_sequence_producer);

size_t zstd_compress_sequences_and_literals(zstd_cctx *cctx, void* dst, size_t dst_capacity,
					    const zstd_sequence *in_seqs, size_t in_seqs_size,
					    const void* literals, size_t lit_size, size_t lit_capacity,
					    size_t decompressed_size)
{
	return ZSTD_compressSequencesAndLiterals(cctx, dst, dst_capacity, in_seqs,
						 in_seqs_size, literals, lit_size,
						 lit_capacity, decompressed_size);
}
EXPORT_SYMBOL(zstd_compress_sequences_and_literals);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Zstd Compressor");
