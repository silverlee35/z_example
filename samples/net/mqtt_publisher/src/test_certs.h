/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TEST_CERTS_H__
#define __TEST_CERTS_H__

#if defined(CONFIG_NET_SOCKETS_OFFLOAD)
/* By default only certificates in DER format are supported. If you want to use
 * certificate in PEM format, you can enable support for it in Kconfig.
 */

#if defined(CONFIG_TLS_CREDENTIAL_FILENAMES)
static const unsigned char ca_certificate[] = "ca_cert.der";
#else
static const unsigned char ca_certificate[] = {
#include "ca_cert.der.inc"
};
#endif

#else
#include <mbedtls/ssl_ciphersuites.h>

#if defined(MBEDTLS_X509_CRT_PARSE_C)
/* This byte array can be generated by
 * "cat ca.crt | sed -e '1d;$d' | base64 -d |xxd -i"
 */
static const unsigned char ca_certificate[] = {
	0x30, 0x82, 0x04, 0x03, 0x30, 0x82, 0x02, 0xeb, 0xa0, 0x03, 0x02, 0x01,
	0x02, 0x02, 0x14, 0x05, 0x8d, 0x61, 0x94, 0x21, 0xaf, 0x76, 0x3e, 0x0d,
	0x84, 0x15, 0xe4, 0x67, 0xfb, 0x8b, 0x51, 0x93, 0x48, 0x2c, 0x0c, 0x30,
	0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x0b,
	0x05, 0x00, 0x30, 0x81, 0x90, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55,
	0x04, 0x06, 0x13, 0x02, 0x47, 0x42, 0x31, 0x17, 0x30, 0x15, 0x06, 0x03,
	0x55, 0x04, 0x08, 0x0c, 0x0e, 0x55, 0x6e, 0x69, 0x74, 0x65, 0x64, 0x20,
	0x4b, 0x69, 0x6e, 0x67, 0x64, 0x6f, 0x6d, 0x31, 0x0e, 0x30, 0x0c, 0x06,
	0x03, 0x55, 0x04, 0x07, 0x0c, 0x05, 0x44, 0x65, 0x72, 0x62, 0x79, 0x31,
	0x12, 0x30, 0x10, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x0c, 0x09, 0x4d, 0x6f,
	0x73, 0x71, 0x75, 0x69, 0x74, 0x74, 0x6f, 0x31, 0x0b, 0x30, 0x09, 0x06,
	0x03, 0x55, 0x04, 0x0b, 0x0c, 0x02, 0x43, 0x41, 0x31, 0x16, 0x30, 0x14,
	0x06, 0x03, 0x55, 0x04, 0x03, 0x0c, 0x0d, 0x6d, 0x6f, 0x73, 0x71, 0x75,
	0x69, 0x74, 0x74, 0x6f, 0x2e, 0x6f, 0x72, 0x67, 0x31, 0x1f, 0x30, 0x1d,
	0x06, 0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x09, 0x01, 0x16,
	0x10, 0x72, 0x6f, 0x67, 0x65, 0x72, 0x40, 0x61, 0x74, 0x63, 0x68, 0x6f,
	0x6f, 0x2e, 0x6f, 0x72, 0x67, 0x30, 0x1e, 0x17, 0x0d, 0x32, 0x30, 0x30,
	0x36, 0x30, 0x39, 0x31, 0x31, 0x30, 0x36, 0x33, 0x39, 0x5a, 0x17, 0x0d,
	0x33, 0x30, 0x30, 0x36, 0x30, 0x37, 0x31, 0x31, 0x30, 0x36, 0x33, 0x39,
	0x5a, 0x30, 0x81, 0x90, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04,
	0x06, 0x13, 0x02, 0x47, 0x42, 0x31, 0x17, 0x30, 0x15, 0x06, 0x03, 0x55,
	0x04, 0x08, 0x0c, 0x0e, 0x55, 0x6e, 0x69, 0x74, 0x65, 0x64, 0x20, 0x4b,
	0x69, 0x6e, 0x67, 0x64, 0x6f, 0x6d, 0x31, 0x0e, 0x30, 0x0c, 0x06, 0x03,
	0x55, 0x04, 0x07, 0x0c, 0x05, 0x44, 0x65, 0x72, 0x62, 0x79, 0x31, 0x12,
	0x30, 0x10, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x0c, 0x09, 0x4d, 0x6f, 0x73,
	0x71, 0x75, 0x69, 0x74, 0x74, 0x6f, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03,
	0x55, 0x04, 0x0b, 0x0c, 0x02, 0x43, 0x41, 0x31, 0x16, 0x30, 0x14, 0x06,
	0x03, 0x55, 0x04, 0x03, 0x0c, 0x0d, 0x6d, 0x6f, 0x73, 0x71, 0x75, 0x69,
	0x74, 0x74, 0x6f, 0x2e, 0x6f, 0x72, 0x67, 0x31, 0x1f, 0x30, 0x1d, 0x06,
	0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x09, 0x01, 0x16, 0x10,
	0x72, 0x6f, 0x67, 0x65, 0x72, 0x40, 0x61, 0x74, 0x63, 0x68, 0x6f, 0x6f,
	0x2e, 0x6f, 0x72, 0x67, 0x30, 0x82, 0x01, 0x22, 0x30, 0x0d, 0x06, 0x09,
	0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03,
	0x82, 0x01, 0x0f, 0x00, 0x30, 0x82, 0x01, 0x0a, 0x02, 0x82, 0x01, 0x01,
	0x00, 0xc1, 0x34, 0x1c, 0xa9, 0x88, 0xcd, 0xf4, 0xce, 0xc2, 0x42, 0x8b,
	0x4f, 0x74, 0xc7, 0x1d, 0xef, 0x8e, 0x6d, 0xd8, 0xb3, 0x6a, 0x63, 0xe0,
	0x51, 0x99, 0x83, 0xeb, 0x84, 0xdf, 0xdf, 0x32, 0x5d, 0x35, 0xe6, 0x06,
	0x62, 0x7e, 0x02, 0x11, 0x76, 0xf2, 0x3f, 0xa7, 0xf2, 0xde, 0xd5, 0x9c,
	0xf1, 0x2d, 0x9b, 0xa1, 0x6e, 0x9d, 0xce, 0xb1, 0xfc, 0x49, 0xd1, 0x5f,
	0xf6, 0xea, 0x37, 0xdb, 0x41, 0x89, 0x03, 0xd0, 0x7b, 0x53, 0x51, 0x56,
	0x4d, 0xed, 0xf1, 0x75, 0xaf, 0xcb, 0x9b, 0x72, 0x45, 0x7d, 0xa1, 0xe3,
	0x91, 0x6c, 0x3b, 0x8c, 0x1c, 0x1c, 0x6a, 0xe4, 0x19, 0x8e, 0x91, 0x88,
	0x34, 0x76, 0xa9, 0x1d, 0x19, 0x69, 0x88, 0x26, 0x6c, 0xaa, 0xe0, 0x2d,
	0x84, 0xe8, 0x31, 0x5b, 0xd4, 0xa0, 0x0e, 0x06, 0x25, 0x1b, 0x31, 0x00,
	0xb3, 0x4e, 0xa9, 0x90, 0x41, 0x62, 0x33, 0x0f, 0xaa, 0x0d, 0xf2, 0xe8,
	0xfe, 0xcc, 0x45, 0x28, 0x1e, 0xaf, 0x42, 0x51, 0x5e, 0x90, 0xc7, 0x82,
	0xca, 0x68, 0xcb, 0x09, 0xb3, 0x70, 0x3c, 0x9c, 0xaa, 0xca, 0x11, 0x66,
	0x3d, 0x6c, 0x22, 0xa3, 0xf3, 0xc3, 0x32, 0xbb, 0x81, 0x4f, 0x33, 0xc7,
	0xdd, 0xc8, 0xa8, 0x06, 0x7a, 0xc9, 0x58, 0xa5, 0xdc, 0xdc, 0xe8, 0xd7,
	0x74, 0xb1, 0x85, 0x24, 0xe7, 0xe3, 0xee, 0x93, 0xf4, 0x8f, 0xf7, 0x6b,
	0xd8, 0xb1, 0xfb, 0xd9, 0xe4, 0xaf, 0xbf, 0x73, 0xd0, 0x40, 0x59, 0x7d,
	0xd0, 0x26, 0x4f, 0x16, 0x1a, 0xc2, 0x51, 0xc4, 0x47, 0x49, 0x2c, 0x68,
	0x13, 0xac, 0xa3, 0x18, 0xe7, 0x67, 0xcf, 0xb7, 0xfa, 0x3e, 0xf7, 0x8b,
	0x20, 0x1e, 0x7b, 0xe2, 0x44, 0x0e, 0x47, 0x0b, 0x7c, 0x78, 0xf9, 0xf4,
	0xca, 0x27, 0x6b, 0x4c, 0x2d, 0x62, 0x72, 0xd8, 0xa4, 0x10, 0x3d, 0xe7,
	0x1d, 0x88, 0x4c, 0x50, 0xe5, 0x02, 0x03, 0x01, 0x00, 0x01, 0xa3, 0x53,
	0x30, 0x51, 0x30, 0x1d, 0x06, 0x03, 0x55, 0x1d, 0x0e, 0x04, 0x16, 0x04,
	0x14, 0xf5, 0x55, 0xeb, 0x10, 0x54, 0x14, 0xf8, 0x86, 0x28, 0x3c, 0xa8,
	0xe5, 0x5d, 0xfe, 0x1d, 0xb8, 0x78, 0x37, 0xd6, 0x12, 0x30, 0x1f, 0x06,
	0x03, 0x55, 0x1d, 0x23, 0x04, 0x18, 0x30, 0x16, 0x80, 0x14, 0xf5, 0x55,
	0xeb, 0x10, 0x54, 0x14, 0xf8, 0x86, 0x28, 0x3c, 0xa8, 0xe5, 0x5d, 0xfe,
	0x1d, 0xb8, 0x78, 0x37, 0xd6, 0x12, 0x30, 0x0f, 0x06, 0x03, 0x55, 0x1d,
	0x13, 0x01, 0x01, 0xff, 0x04, 0x05, 0x30, 0x03, 0x01, 0x01, 0xff, 0x30,
	0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x0b,
	0x05, 0x00, 0x03, 0x82, 0x01, 0x01, 0x00, 0x66, 0xbd, 0x91, 0x2d, 0xb5,
	0x37, 0xbd, 0x13, 0x84, 0xce, 0xbf, 0x1e, 0x3f, 0x43, 0xee, 0x66, 0xd5,
	0xc4, 0xa2, 0xc1, 0x8d, 0x55, 0x9e, 0xd9, 0x33, 0xec, 0x19, 0xf6, 0xe5,
	0xde, 0xb1, 0x03, 0x7d, 0x9f, 0x8e, 0x29, 0x16, 0x76, 0x8f, 0xa0, 0x02,
	0xea, 0xbe, 0xe3, 0x6f, 0x84, 0xd9, 0x3b, 0x77, 0x73, 0x17, 0x6a, 0x7a,
	0x76, 0x06, 0xeb, 0x95, 0x4e, 0xf5, 0x63, 0xfe, 0x0a, 0xd1, 0x37, 0x73,
	0x22, 0x34, 0x63, 0xdd, 0xc4, 0x37, 0x29, 0x29, 0xb8, 0xd4, 0x9b, 0xd4,
	0x43, 0x48, 0x59, 0xfd, 0xcd, 0x38, 0x88, 0x60, 0xe0, 0xff, 0x15, 0x9f,
	0xfa, 0x9a, 0x79, 0xf2, 0x77, 0xcf, 0x01, 0x8c, 0x2e, 0x7a, 0xba, 0xee,
	0x3c, 0xd5, 0xa6, 0x95, 0x2b, 0x56, 0x01, 0x77, 0xf4, 0x51, 0x3a, 0x91,
	0xb6, 0x0e, 0x21, 0x40, 0x35, 0x81, 0xb9, 0x41, 0x43, 0x25, 0x3b, 0x96,
	0xba, 0xe0, 0x6f, 0x11, 0x7b, 0x9d, 0xcf, 0xbe, 0x1e, 0x87, 0xfc, 0x0a,
	0xb0, 0xcc, 0x1f, 0xbb, 0x51, 0xc5, 0xbe, 0x3c, 0xb9, 0x67, 0x48, 0x8c,
	0x0d, 0x4f, 0x0f, 0x50, 0x37, 0xa9, 0x8d, 0x5a, 0x25, 0x38, 0x2b, 0x9e,
	0xf5, 0xab, 0x21, 0x95, 0x2e, 0x04, 0x07, 0x92, 0x04, 0x09, 0xd4, 0x91,
	0xd9, 0x32, 0x2d, 0x9c, 0x02, 0x22, 0x23, 0x08, 0xa6, 0xc7, 0xcd, 0xfd,
	0x2d, 0xd5, 0x1d, 0x46, 0xe7, 0x5a, 0x7c, 0xcb, 0xb9, 0x4f, 0x95, 0xe6,
	0x6b, 0x5f, 0x36, 0x38, 0x2d, 0x3f, 0xbb, 0xfc, 0x51, 0x94, 0x49, 0xbe,
	0xb6, 0xf2, 0x86, 0x1a, 0x67, 0xc5, 0x70, 0xdd, 0x29, 0x8a, 0xa5, 0x65,
	0xf0, 0xea, 0xd2, 0x3c, 0x18, 0x08, 0x95, 0xbf, 0xb5, 0x20, 0xa2, 0x44,
	0x9b, 0xf5, 0xeb, 0x89, 0x6a, 0xff, 0x0a, 0xae, 0x21, 0xfc, 0x97, 0xc1,
	0xec, 0xd4, 0xec, 0x7b, 0x35, 0x6c, 0x96, 0x09, 0x01, 0x6a, 0x85
};
#endif /* MBEDTLS_X509_CRT_PARSE_C */

#if defined(MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED)
/* Avoid leading zero in psk because there's a potential issue of mosquitto
 * that leading zero of psk will be skipped and it leads to TLS handshake
 * failure
 */
const unsigned char client_psk[] = {
	0x01, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
};

const char client_psk_id[] = "Client_identity";
#endif

#endif /* CONFIG_NET_SOCKETS_OFFLOAD */

#endif
