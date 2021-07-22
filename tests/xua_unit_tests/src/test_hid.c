// Copyright 2021 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#include <stddef.h>
#include <stdio.h>

#include "xua_unit_tests.h"
#include "xua_hid_report_descriptor.h"
#include "hid_report_descriptor.h"

#define HID_REPORT_ITEM_TYPE_GLOBAL     ( 0x01 )
#define HID_REPORT_ITEM_TYPE_LOCAL      ( 0x02 )
#define HID_REPORT_ITEM_TYPE_MAIN       ( 0x00 )
#define HID_REPORT_ITEM_TYPE_RESERVED   ( 0x03 )

#define CONSUMER_CONTROL_PAGE   ( 0x0C )
#define LOUDNESS_CONTROL        ( 0xE7 )

static unsigned construct_usage_header( unsigned size )
{
    unsigned header = 0x00;

    header |= ( HID_REPORT_ITEM_USAGE_TAG  << HID_REPORT_ITEM_HDR_TAG_SHIFT  ) & HID_REPORT_ITEM_HDR_TAG_MASK;
    header |= ( HID_REPORT_ITEM_USAGE_TYPE << HID_REPORT_ITEM_HDR_TYPE_SHIFT ) & HID_REPORT_ITEM_HDR_TYPE_MASK;

    header |= ( size << HID_REPORT_ITEM_HDR_SIZE_SHIFT ) & HID_REPORT_ITEM_HDR_SIZE_MASK;

    return header;
}

void setUp( void )
{
    hidResetReportDescriptor();
}

// Basic report descriptor tests
void test_unprepared_hidGetReportDescriptor( void )
{
    unsigned char* reportDescPtr = hidGetReportDescriptor();
    TEST_ASSERT_NULL( reportDescPtr );

    unsigned reportLength = hidGetReportLength();
    TEST_ASSERT_EQUAL_UINT( 0, reportLength );
}

void test_prepared_hidGetReportDescriptor( void )
{
    hidPrepareReportDescriptor();
    unsigned char* reportDescPtr = hidGetReportDescriptor();
    TEST_ASSERT_NOT_NULL( reportDescPtr );

    unsigned reportLength = hidGetReportLength();
    TEST_ASSERT_EQUAL_UINT( HID_REPORT_LENGTH, reportLength );
}

void test_reset_unprepared_hidGetReportDescriptor( void )
{
    hidPrepareReportDescriptor();
    hidResetReportDescriptor();
    unsigned char* reportDescPtr = hidGetReportDescriptor();
    TEST_ASSERT_NULL( reportDescPtr );
}

void test_reset_prepared_hidGetReportDescriptor( void )
{
    hidPrepareReportDescriptor();
    hidResetReportDescriptor();
    hidPrepareReportDescriptor();
    unsigned char* reportDescPtr = hidGetReportDescriptor();
    TEST_ASSERT_NOT_NULL( reportDescPtr );
}

// Basic item tests
void test_max_loc_hidGetReportItem( void )
{
    const unsigned bit = MAX_VALID_BIT;
    const unsigned byte = MAX_VALID_BYTE;
    unsigned char data[ HID_REPORT_ITEM_MAX_SIZE ];
    unsigned char header;
    unsigned char page;

    unsigned retVal = hidGetReportItem( byte, bit, &page, &header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );
    TEST_ASSERT_EQUAL_UINT( CONSUMER_CONTROL_PAGE, page );
    TEST_ASSERT_EQUAL_UINT( 0x09, header );
    TEST_ASSERT_EQUAL_UINT( 0xEA, data[ 0 ]);
    TEST_ASSERT_EQUAL_UINT( 0x00, data[ 1 ]);
}

void test_min_loc_hidGetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    unsigned char data[ HID_REPORT_ITEM_MAX_SIZE ];
    unsigned char header;
    unsigned char page;

    unsigned retVal = hidGetReportItem( byte, bit, &page, &header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );
    TEST_ASSERT_EQUAL_UINT( CONSUMER_CONTROL_PAGE, page );
    TEST_ASSERT_EQUAL_UINT( 0x09, header );
    TEST_ASSERT_EQUAL_UINT( 0xE2, data[ 0 ]);
    TEST_ASSERT_EQUAL_UINT( 0x00, data[ 1 ]);
}

void test_overflow_bit_hidGetReportItem( void )
{
    const unsigned bit = MAX_VALID_BIT + 1;
    const unsigned byte = MAX_VALID_BYTE;
    unsigned char data[ HID_REPORT_ITEM_MAX_SIZE ] = { 0xBA, 0xD1 };
    unsigned char header = 0xAA;
    unsigned char page = 0x44;

    unsigned retVal = hidGetReportItem( byte, bit, &page, &header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_LOCATION, retVal );
    TEST_ASSERT_EQUAL_UINT( 0x44, page );
    TEST_ASSERT_EQUAL_UINT( 0xAA, header );
    TEST_ASSERT_EQUAL_UINT( 0xBA, data[ 0 ]);
    TEST_ASSERT_EQUAL_UINT( 0xD1, data[ 1 ]);
}

void test_overflow_byte_hidGetReportItem( void )
{
    const unsigned bit = MAX_VALID_BIT;
    const unsigned byte = MAX_VALID_BYTE + 1;
    unsigned char data[ HID_REPORT_ITEM_MAX_SIZE ] = { 0xBA, 0xD1 };
    unsigned char header = 0xAA;
    unsigned char page = 0x44;

    unsigned retVal = hidGetReportItem( byte, bit, &page, &header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_LOCATION, retVal );
    TEST_ASSERT_EQUAL_UINT( 0x44, page );
    TEST_ASSERT_EQUAL_UINT( 0xAA, header );
    TEST_ASSERT_EQUAL_UINT( 0xBA, data[ 0 ]);
    TEST_ASSERT_EQUAL_UINT( 0xD1, data[ 1 ]);
}

void test_underflow_bit_hidGetReportItem( void )
{
    const      int  bit = MIN_VALID_BIT - 1;
    const unsigned byte = MIN_VALID_BYTE;
    unsigned char data[ HID_REPORT_ITEM_MAX_SIZE ] = { 0xBA, 0xD1 };
    unsigned char header = 0xAA;
    unsigned char page = 0x44;

    unsigned retVal = hidGetReportItem( byte, ( unsigned ) bit, &page, &header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_LOCATION, retVal );
    TEST_ASSERT_EQUAL_UINT( 0x44, page );
    TEST_ASSERT_EQUAL_UINT( 0xAA, header );
    TEST_ASSERT_EQUAL_UINT( 0xBA, data[ 0 ]);
    TEST_ASSERT_EQUAL_UINT( 0xD1, data[ 1 ]);
}

void test_underflow_byte_hidGetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const      int byte = MIN_VALID_BYTE - 1;
    unsigned char data[ HID_REPORT_ITEM_MAX_SIZE ] = { 0xBA, 0xD1 };
    unsigned char header = 0xAA;
    unsigned char page = 0x44;

    unsigned retVal = hidGetReportItem(( unsigned ) byte, bit, &page, &header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_LOCATION, retVal );
    TEST_ASSERT_EQUAL_UINT( 0x44, page );
    TEST_ASSERT_EQUAL_UINT( 0xAA, header );
    TEST_ASSERT_EQUAL_UINT( 0xBA, data[ 0 ]);
    TEST_ASSERT_EQUAL_UINT( 0xD1, data[ 1 ]);
}

// Configurable and non-configurable item tests
void test_configurable_item_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char data[ 1 ] = { LOUDNESS_CONTROL };
    const unsigned char header = construct_usage_header( sizeof data / sizeof( unsigned char ));
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );
}

void test_nonconfigurable_item_hidSetReportItem( void )
{
    const unsigned bit = MAX_VALID_BIT;     // This bit and byte combination should not appear in the
    const unsigned byte = MIN_VALID_BYTE;    // hidConfigurableItems list in hid_report_descriptors.c.
    const unsigned char data[ 1 ] = { LOUDNESS_CONTROL };
    const unsigned char header = construct_usage_header( sizeof data / sizeof( unsigned char ));
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_LOCATION, retVal );
}

// Bit range tests
void test_max_bit_hidSetReportItem( void )
{
    const unsigned bit = MAX_VALID_BIT;     // Only byte 1 has bit 7 not reserved,  See the
    const unsigned byte = MAX_VALID_BYTE;   // hidConfigurableItems list in hid_report_descriptors.c.
    const unsigned char header = construct_usage_header( 0 );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );
}

void test_min_bit_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char header = construct_usage_header( 0 );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );
}

void test_overflow_bit_hidSetReportItem( void )
{
    const unsigned bit = MAX_VALID_BIT + 1;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char header = construct_usage_header( 0 );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_LOCATION, retVal );
}

void test_underflow_bit_hidSetReportItem( void )
{
    const int bit = MIN_VALID_BIT - 1;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char header = construct_usage_header( 0 );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, ( unsigned ) bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_LOCATION, retVal );
}

// Byte range tests
void test_max_byte_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MAX_VALID_BYTE;
    const unsigned char header = construct_usage_header( 0 );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );
}

void test_min_byte_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char header = construct_usage_header( 0 );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );
}

void test_overflow_byte_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MAX_VALID_BYTE + 1;
    const unsigned char header = construct_usage_header( 0 );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_LOCATION, retVal );
}

void test_underflow_byte_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const int byte = MIN_VALID_BYTE - 1;
    const unsigned char header = construct_usage_header( 0 );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( ( unsigned ) byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_LOCATION, retVal );
}

// Size range tests
void test_max_size_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char data[ HID_REPORT_ITEM_MAX_SIZE ] = { 0x00 };
    const unsigned char header = construct_usage_header( HID_REPORT_ITEM_MAX_SIZE );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );
}

void test_min_size_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char header = construct_usage_header( 0x00 );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );
}

void test_unsupported_size_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char header = construct_usage_header( 0x03 );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_HEADER, retVal );
}

// Header tag and type tests
void test_bad_tag_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char good_header = construct_usage_header( 0x00 );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    for( unsigned tag = 0x01; tag <= 0x0F; ++tag ) {
        unsigned char bad_header = good_header | (( 0x0F << HID_REPORT_ITEM_HDR_TAG_SHIFT ) & HID_REPORT_ITEM_HDR_TAG_MASK );
        unsigned retVal = hidSetReportItem( byte, bit, page, bad_header, NULL );
        TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_HEADER, retVal );
    }
}

void test_global_type_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char header = ( construct_usage_header( 0x00 ) & ~HID_REPORT_ITEM_HDR_TYPE_MASK ) |
        (( HID_REPORT_ITEM_TYPE_GLOBAL << HID_REPORT_ITEM_HDR_TYPE_SHIFT ) & HID_REPORT_ITEM_HDR_TYPE_MASK );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_HEADER, retVal );
}

void test_local_type_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char header = ( construct_usage_header( 0x00 ) & ~HID_REPORT_ITEM_HDR_TYPE_MASK ) |
        (( HID_REPORT_ITEM_TYPE_LOCAL << HID_REPORT_ITEM_HDR_TYPE_SHIFT ) & HID_REPORT_ITEM_HDR_TYPE_MASK );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );
}

void test_main_type_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char header = ( construct_usage_header( 0x00 ) & ~HID_REPORT_ITEM_HDR_TYPE_MASK ) |
        (( HID_REPORT_ITEM_TYPE_MAIN << HID_REPORT_ITEM_HDR_TYPE_SHIFT ) & HID_REPORT_ITEM_HDR_TYPE_MASK );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_HEADER, retVal );
}

void test_reserved_type_hidSetReportItem( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char header = ( construct_usage_header( 0x00 ) & ~HID_REPORT_ITEM_HDR_TYPE_MASK ) |
        (( HID_REPORT_ITEM_TYPE_RESERVED << HID_REPORT_ITEM_HDR_TYPE_SHIFT ) & HID_REPORT_ITEM_HDR_TYPE_MASK );
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, NULL );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_BAD_HEADER, retVal );
}

// Combined function tests
void test_initial_modification_without_subsequent_preparation( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char data[ 1 ] = { LOUDNESS_CONTROL };
    const unsigned char header = construct_usage_header( sizeof data / sizeof( unsigned char ));
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );

    unsigned char* reportDescPtr = hidGetReportDescriptor();
    TEST_ASSERT_NULL( reportDescPtr );
}

void test_initial_modification_with_subsequent_preparation( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char data[ 1 ] = { LOUDNESS_CONTROL };
    const unsigned char header = construct_usage_header( sizeof data / sizeof( unsigned char ));
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    unsigned retVal = hidSetReportItem( byte, bit, page, header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );

    hidPrepareReportDescriptor();
    unsigned char* reportDescPtr = hidGetReportDescriptor();
    TEST_ASSERT_NOT_NULL( reportDescPtr );
}

void test_initial_modification_with_subsequent_verification( void )
{
    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;

    unsigned char get_data[ HID_REPORT_ITEM_MAX_SIZE ] = { 0xFF, 0xFF };
    unsigned char get_header = 0xFF;
    unsigned char get_page = 0xFF;

    const unsigned char set_data[ 1 ] = { LOUDNESS_CONTROL };
    const unsigned char set_header = construct_usage_header( sizeof set_data / sizeof( unsigned char ));
    const unsigned char set_page = CONSUMER_CONTROL_PAGE;

    unsigned setRetVal = hidSetReportItem( byte, bit, set_page, set_header, set_data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, setRetVal );

    unsigned getRetVal = hidGetReportItem( byte, bit, &get_page, &get_header, get_data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, getRetVal );
    TEST_ASSERT_EQUAL_UINT( get_page, set_page );
    TEST_ASSERT_EQUAL_UINT( get_header, set_header );
    TEST_ASSERT_EQUAL_UINT( get_data[ 0 ], set_data[ 0 ]);
    TEST_ASSERT_EQUAL_UINT( get_data[ 1 ], set_data[ 1 ]);
}

void test_modification_without_subsequent_preparation( void )
{
    hidPrepareReportDescriptor();
    unsigned char* reportDescPtr = hidGetReportDescriptor();
    TEST_ASSERT_NOT_NULL( reportDescPtr );

    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char data[ 1 ] = { LOUDNESS_CONTROL };
    const unsigned char header = construct_usage_header( sizeof data / sizeof( unsigned char ));
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    hidResetReportDescriptor();
    unsigned retVal = hidSetReportItem( byte, bit, page, header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );

    reportDescPtr = hidGetReportDescriptor();
    TEST_ASSERT_NULL( reportDescPtr );
}

void test_modification_with_subsequent_preparation( void )
{
    hidPrepareReportDescriptor();
    unsigned char* reportDescPtr = hidGetReportDescriptor();
    TEST_ASSERT_NOT_NULL( reportDescPtr );

    const unsigned bit = MIN_VALID_BIT;
    const unsigned byte = MIN_VALID_BYTE;
    const unsigned char data[ 1 ] = { LOUDNESS_CONTROL };
    const unsigned char header = construct_usage_header( sizeof data / sizeof( unsigned char ));
    const unsigned char page = CONSUMER_CONTROL_PAGE;

    hidResetReportDescriptor();
    unsigned retVal = hidSetReportItem( byte, bit, page, header, data );
    TEST_ASSERT_EQUAL_UINT( HID_STATUS_GOOD, retVal );

    hidPrepareReportDescriptor();
    reportDescPtr = hidGetReportDescriptor();
    TEST_ASSERT_NOT_NULL( reportDescPtr );
}