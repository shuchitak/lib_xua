// Copyright 2021 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/**
 * @brief Human Interface Device (HID) Report descriptor
 *
 * This file defines the structure of the HID Report descriptor and declares
 *   functions for manipulating it.
 * Because the Report descriptor defines the length of the HID Report, this file
 *   declares a function for obtaining the Report length as well.
 * The using application has the responsibility to define the report descriptor
 *   structure and default contents in their hid_report_descriptor.h file.
 * Document section numbers refer to the HID Device Class Definition, version 1.11.
 */

#ifndef _HID_REPORT_DESCRIPTOR_
#define _HID_REPORT_DESCRIPTOR_

#include <stddef.h>

#define HID_REPORT_ITEM_HDR_SIZE_MASK   ( 0x03 )
#define HID_REPORT_ITEM_HDR_SIZE_SHIFT  ( 0 )

#define HID_REPORT_ITEM_HDR_TAG_MASK    ( 0xF0 )
#define HID_REPORT_ITEM_HDR_TAG_SHIFT   ( 4 )

#define HID_REPORT_ITEM_HDR_TYPE_MASK   ( 0x0C )
#define HID_REPORT_ITEM_HDR_TYPE_SHIFT  ( 2 )

#define HID_REPORT_ITEM_LOC_BIT_MASK    ( 0x70 )
#define HID_REPORT_ITEM_LOC_BIT_SHIFT   ( 4 )

#define HID_REPORT_ITEM_LOC_BYTE_MASK   ( 0x0F )
#define HID_REPORT_ITEM_LOC_BYTE_SHIFT  ( 0 )

#define HID_REPORT_ITEM_MAX_SIZE        ( 2 )

#define HID_REPORT_ITEM_USAGE_TAG       ( 0 )
#define HID_REPORT_ITEM_USAGE_TYPE      ( 2 )

#define HID_STATUS_GOOD                 ( 0 )
#define HID_STATUS_BAD_HEADER           ( 1 )
#define HID_STATUS_BAD_LOCATION         ( 2 )
#define HID_STATUS_BAD_PAGE             ( 3 )
#define HID_STATUS_IN_USE               ( 4 )

/**
 * @brief USB HID Report Descriptor. Short Item
 *
 * @note
 * To reduce memory use, this type does not support Short Items with 4 data bytes.
 * See section 6.2.2.2
 *
 * Elements:
 *
 *   header   - the item prefix containing the size, type and tag fields (see 6.2.2.2)
 *              Format (bit range): bSize (0:1), bType (2:3), bTag (4:7)
 *   data     - a two byte array for holding the item's data
 *              The bSize field indicates which data bytes are in use
 *   location - a non-standard extension locating the item within the HID Report
 *              Format (bit range): iByte (0:3), iBit (4:6), Reserved (7)
 */
typedef struct
{
    unsigned char header;
    unsigned char data[ HID_REPORT_ITEM_MAX_SIZE ];
    unsigned char location;
} USB_HID_Short_Item_t;

/**
 * @brief Get the HID Report descriptor
 *
 * This function returns a pointer to the USB HID Report descriptor.
 * It returns NULL if the Report descriptor has not been prepared,
 *   i.e., no one has called \c hidPrepareReportDescriptor().
 *
 * @note An XC-callable version of this function has not been provided.
 *       XC requires explicit declaration of the kind of pointer returned,
 *       hence an XC implementation of the function.
 *
 * @return A pointer to a list of unsigned char containing the Report descriptor
 */
#if !defined(__XC__)
unsigned char* hidGetReportDescriptor( void );
#endif

/**
 * @brief Get the length of the HID Report descriptor
 *
 * This function returns the length of the USB HID Report descriptor.
 * It returns zero if the Report descriptor has not been prepared,
 *   i.e., no one has called \c hidPrepareReportDescriptor().
 *
 * @return The length of the Report descriptor in bytes
 */
size_t hidGetReportDescriptorLength( void );

/**
 * @brief Get a HID Report descriptor item
 *
 * Parameters:
 *
 *  @param[in]  byte    The byte position of the control within the HID Report
 *  @param[in]  bit     The bit position of the control within the \a byte
 *  @param[out] page    The USB HID Usage Page code for the Item (see 5.5)
 *  @param[out] header  The LSB of the Item containing the bSize, bType and bTag fields (see 6.2.2.2)
 *  @param[out] data    A two element array containing data bytes for the Item
 *
 * @return A status value
 * @retval \c HID_STATUS_GOOD           Item successfully returned
 * @retval \c HID_STATUS_BAD_LOCATION   The \a bit or \a byte arguments specify a location outside
 *                                      of the HID Report
 */
#if defined(__XC__)
unsigned hidGetReportItem( const unsigned byte, const unsigned bit, unsigned char* unsafe const page, unsigned char* unsafe const header, unsigned char* unsafe const data);
#else
unsigned hidGetReportItem( const unsigned byte, const unsigned bit, unsigned char* const page, unsigned char* const header, unsigned char data[]);
#endif

/**
 * @brief Get the length of the HID Report
 *
 * This function returns the length of the USB HID Report.
 * It returns zero if the Report descriptor has not been prepared,
 *   i.e., no one has called \c hidPrepareReportDescriptor().
 *
 * @return The length of the Report in bytes
 */
size_t hidGetReportLength( void );

/**
 * @brief Prepare the USB HID Report descriptor
 *
 * After preparation, \c hidGetReportDescriptor() returns a list suitable for transmission over USB.
 * Call this function after altering one or more Report Items using \c hidSetReportItem().
 */
void hidPrepareReportDescriptor( void );

/**
 * @brief Reset the USB HID Report descriptor
 *
 * After reset, \c hidGetReportDescriptor() returns NULL until a subsequent call to
 *   \c hidPrepareReportDescriptor() occurs.
 * Call this function before altering one or more Report Items using \c hidSetReportItem().
 */
void hidResetReportDescriptor( void );

/**
 * @brief Modify a HID Report descriptor item
 *
 * @warning This function does not check that the length of the \a data array matches the value of
 *          the bSize field in the \a header.  For safe operation use a \a data array of at least
 *          \c HID_REPORT_ITEM_MAX_SIZE bytes in length.
 *
 * Parameters:
 *
 *  @param[in] byte     The byte position of the control within the HID Report
 *  @param[in] bit      The bit position of the control within the \a byte
 *  @param[in] page     The USB HID Usage Page code for the Item (see 5.5)
 *  @param[in] header   The LSB of the Item containing the bSize, bType and bTag fields (see 6.2.2.2)
 *  @param[in] data     An array containing data bytes or NULL for an Item with no data
 *
 * @return A status value
 * @retval \c HID_STATUS_GOOD           Item successfully updated
 * @retval \c HID_STATUS_BAD_HEADER     The Item header specified a data size greater than 2 or
 *                                      a Tag or Type inconsistent with a Usage Item
 * @retval \c HID_STATUS_BAD_LOCATION   The \a bit or \a byte arguments specify a location outside
 *                                      of the HID Report
 * @retval \c HID_STATUS_BAD_PAGE       The \a byte argument specifies a location for controls from
 *                                      a Usage Page other than the one given by the \a page parameter
 * @retval \c HID_STATUS_IN_USE         The Report descriptor is in use
 */
unsigned hidSetReportItem( const unsigned byte, const unsigned bit, const unsigned char page, const unsigned char header, const unsigned char data[]);

#endif // _HID_REPORT_DESCRIPTOR_