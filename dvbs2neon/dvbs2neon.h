#ifndef DVB_NEON
#define DVB_NEON

#define VERSION 0x0042 		// v0.40

#define INFO_STRING			"dvbs2neon0v42; TS to DVB-S2 encapsulator; normal and short frames"

#define TIMING				0					//	for development only

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//	dvbs2neon.S
//
//	(C) Brian Jordan, G4EWJ, April-December 2020, g4ewj at yahoo dot com
//
//	Transport stream packet to DVB-S2 frame converter for ARM 
//	Accepts 188 byte packets and produces DVB-S2 frames, as symbols to send to a modulator
//	Uses Neon instructions, so will not run on an RPi Zero
//	Supports DVB-S2 multistream (untested)
//	Supports all Normal frame and Short frame FECs
//  Supports rolloff 0.35, 0.25, 0.20 (also 0.15, although not in the DVB-S2 spec)
//	Supports pilot tones
//
//	For FEC 3/4 APSK16 symbol output, the approximate Normal frame processing time is 220us on an RPi3 
//
//	The last BB frame may be returned
//	A supplied BB frame may be converted to symbols
//
//	Parts derived from DATV-Express and RPiDATV
//	Thanks to G4GUO and F5OEO for their help
//
//	This software is provided free for non-commercial amateur radio use.
//	It has no warranty of fitness for any purpose.
//	Generation of a DVB-S2 transport stream may require a licence.
//	Use at your own risk.
//
//	If @ is not the comment character for your assembler, search and replace // with yours.
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//	Change history
//
//	dvbs2neon versions:
//
//	2022-05-03	v0.40	get last BB frame function
//						accept BB frame and convert to symbols
//	2020-12-30	v0.30	added short frames
//						added frame type, constellation and FEC in output metadata
//						removed FIR filtering
//						*** v0.30 is used by some versions of F5OEO's Pluto software
//  2020-12-14  v0.25   corrected number of pilot blocks for QPSK
//                      corrected symbol count in output data for QPSK with pilots
//                      corrected efficiencies for QPSK with pilots
//	2020-04-03	v0.24	added FIR fast table filtering for QPSK, rolloff 0.35, 16x upsample, 513 taps
//						removed IQ output format
//						check for symbol output complete at the start of the inner loop
//						  to prevent the output buffers overflowing too much 
//						added upsample values and output format to the meta data before the returned pointer
//	2020-03-29	v0.22	for symbols output, put the 2 byte symbol count at the start of the output buffer
//						the pointer returned is still the start of the symbols
//						so the symbol count is *(uint16*)((uint32)pointer-2)	
//	2020-03-28	v0.21	returns the correct frame output buffer address for symbol output (base+2)	
//	2020-03-28	v0.20	first major release - see above for functionality	
//	2019-10-21	v0.10	first beta test release - not tested live - FECs 1/4, 3/4, 5/6	
//
//	dvbs2arm versions:
//
//	2019-09-29	v1.32	rounded up efficiency values
//	2019-09-15	v1.31	added FEC 5/6
//	2018-04-18	v1.30	corrected rolloff value in BB header
//	2018-02-22	v1.29	corrected some typos and added more comments
//	2018-02-12	v1.28	added FEC 1/4, made rolloff selectable, added efficiency calculation
//	2018-02-02	v1.07	symbol scrambler/splitter speed increased
//						corrected some typos and added more comments
//	2018-01-28	v1.01	first release
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//	External entry points
//
//	Only these external entry points are required 
//	The calling requirements for other routines are shown for information only
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//.global dvbs2neon_control
//.global dvbs2neon_packet
	

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
// parameters structure (address passed by CONTROL_SET_PARAMETERS)
//
// this is compatible with the parameter structure of DATV-Express
// DVB2FrameFormat	fmt ;
//
// the parameter values are also compatible
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

typedef signed int			int32 ;
typedef unsigned int		uint32 ;
typedef unsigned char		uchar ;

extern "C"
{
extern signed int	dvbs2neon_control 				(uint32, uint32, uint32, uint32) ;
extern signed int	dvbs2neon_set_output_buffer		(uint32, uint32, uint32) ;
extern signed int	dvbs2neon_packet 				(uint32, uint32, uint32) ;
}	

typedef struct 
{
	uint32				frame_type ;
	uint32				fec ;
	uint32				roll_off ;
	uint32				constellation ;
	uint32				pilots ;
	uint32				output_format ;		// dummy_frame in DATV-Express
} DVB2FrameFormat ;


// offsets for the calling parameters block

/*#define FRAME_TYPE_PAR		0
#define FEC_PAR				4
#define ROLL_OFF_PAR		8
#define CONSTELLATION_PAR	12
#define PILOTS_PAR			16
#define OUTPUT_FORMAT_PAR	20	
*/
// offsets for the calling parameters block

#define FRAME_TYPE_PAR		0
#define FEC_PAR				4
#define ROLL_OFF_PAR		8
#define CONSTELLATION_PAR	12
#define PILOTS_PAR			16
#define DUMMY_FRAME_PAR		20	
#define OUTPUT_FORMAT_PAR	24	
	
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//	dvbs2neon_control 
//
//	Calling from C:
//		int32 dvbs2neon_control (uint32 stream, uint32 command, uint32 param1, uint32 param2) ;
//
//	Accepts a stream number, a command and 2 parameters
//	Returns a status indication; < 0 is an error
//	All registers except r0 are restored to their original values
//
//	command name		stream	command param1					param2			return 
//						(r0)	(r1)	(r2)					(r3)			(<0 = error)
//	--------------------------------------------------------------------------------------------
//	CONTROL_RESET_FULL			0		buffer pool address		buffer size		version		   
//	CONTROL_RESET_STREAM		1		none					none			0		 
//	CONTROL_SET_PARAMETERS		2		param block address		none			0
//	CONTROL_SET_OUTPUT_BUFFER	3		buffer address			none			0
//	CONTROL_GET_EFFICIENCY		4		none					none			decimal value
//	CONTROL_GET_INFO			5		none					none			info string pointer 
//	CONTROL_GET_LAST_BBFRAME	6		user buffer pointer		none			pointer to start of BB frame data
//	CONTROL_CONVERT_BBFRAME		7		input buffer pointer	output . . .	pointer to start of symbols	
//
//	Version 12.34 would be returned as 0x00001234
//
//	When resetting a stream, normal frame, FEC 2/3, rolloff 0.35 and pilots off are selected as defaults
//
//	The recommended size for the supplied buffer pool is 4 * 36 * 1024 bytes (512 byte alignment recommended)
//	The recommended size for individual buffers is 36 * 1024 bytes (128 byte alignment recommended)
//  
//	This area is used to cycle around 4 output buffers
//	It is often in non-cached ram used for DMA
//	Alternatively, output buffers for each stream may be managed manually using the SET_OUTPUT_BUFFER command
//
//	Parameters can be changed on the fly without any loss of data
//
//	The effiency value is used to determine the required bit rate of the incoming 188 byte TS
//	Bit rate = symbol_rate * efficiency / 1,000,000
//	
//	Stream zero is used for non-stream mode
//	Typical usage for non-stream mode:
//
//	#define STREAM0		0
//
//	DVB2FrameFormat		fmt ;
//
//	fmt.frame_type		= FRAME_NORMAL
//	fmt.fec				= CR_3_4 ;
//	fmt.roll_off		= RO_0_20 ;
//	fmt.constellation	= M_16APSK ;
//	fmt.pilots			= PILOTS_ON ;
//	fmt.dummy_frame		= OUTPUT_FORMAT_SYMBOLS ;
//
//	dvbs2neon_control (0,CONTROL_RESET_FULL,&buffer144k,sizeof(buffer144k)) ;	// pass output buffer pool address and size and completely reset
//	dvbs2neon_control (STREAM0,CONTROL_RESET_STREAM,0,0) ;						// reset stream zero - clear data and set normal frame, FEC35, rolloff 0.35 
//	dvbs2neon_control (STREAM0,CONTROL_SET_PARAMETERS,(uint32)&fmt) ;			// set parameters
//
//	CONTROL_SET_PARAMETERS does not clear any data, so frame type, constellation and FEC can be changed on the fly
//
//	while (1)
//	{
//		outbuffer = dvbs2neon_packet (STREAM0,&packet188,0) ;
//		if (outbuffer)
//		{
//			transmit (outbuffer) ;
//			outbuffer = 0 ;
//		}
//	} ;
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//	Metadata returned when a frame is returned as symbols
//
//	uchar	unused0 ;
//	uchar	unused1 ;
//	uchar	unused2 ;
//  uchar	frame_type ;		// 0, 0x10 ==> NORMAL, SHORT
//	uchar	constellation ;		// 0 - 3   ==> QPSK, 8PSK, 16APSK, 32APSK
//	uchar	fec	;				// 0 - 10  ==> 1/4 - 9/10
//	uchar	unused3 ;
//	uchar	output_format ;		// 0       ==> OUTPUT_FORMAT_SYMBOLS (1 byte each)
//	uint16	symbol_count ;		// the number of symbols that the output relates to
//	uchar	data[] ;			// *** the pointer returned always points to the actual data here ***
//
//	p = dvbs2neon_packet (...) ;
//	if ((int)p > 0)	 // frame ready
//	{
//		p16 			= (uint16*) p ;
//		symbol_count 	= p16 [-1] ;		
//
//		p8 = (uchar*) p ;
//		output_format	= p8 [-3] ;
//		fec				= p8 [-5] ;
//		constellation	= p8 [-6] ;
//		frame_type		= p8 [-7] ;
//	} 
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//	Metadata returned when a BB frame is converted to symbols
//
//		same as the section above
//
//	mysymbolsbuffer_0x8400 should be minimum size 0x8400 and on a 128 byte boundary
//
//	p = dvbs2neon_control (0, CONTROL_CONVERT_BBFRAME, &mybbframe, &mysymbolsbuffer_0x8400) ;
//
//	The parameters for STREAM0 are always used, so these should be set beforehand
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//	Metadata returned when a BB frame is returned
//
//	uchar	unused100[6] ;
//	uchar	unused0 ;
//	uchar	unused1 ;
//	uchar	unused2 ;
//  uchar	frame_type ;		// 0, 0x10 ==> NORMAL, SHORT
//	uchar	unused102 ;
//	uchar	fec	;				// 0 - 10  ==> 1/4 - 9/10
//	uchar	unused3 ;
//	uchar	output_format ;		// 9       ==> OUTPUT_FORMAT_BBFRAME (bytes)
//	uint16	byte_count ;		// the number of symbols that the output relates to
//	uchar	data[] ;			// *** the pointer returned always points to the actual data here ***
//
//	mybuffer_0x2100 should be minimum size 0x2100 and on a 128 byte boundary
//
//	p = dvbs2neon_control (STREAM0,CONTROL_GET_LAST_BBFRAME, &mybuffer_0x2100, 0) ;
//	if ((int)p > 0)	 	// valid
//	{
//		p16 			= (uint16*) p ;
//		symbol_count 	= p16 [-1] ;		
//
//		p8 = (uchar*) p ;
//		output_format	= p8 [-3] ;
//		fec				= p8 [-5] ;
//		frame_type		= p8 [-7] ;
//	} 
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//	Meta data returned when a BBframe is returned:
//
//	uchar	unused0 ;
//	uchar	unused1 ;
//	uchar	unused2 ;
//  uchar	frame_type			// 0, 0x10 ==> NORMAL, SHORT
//	uchar	unused4 ;	
//	uchar	fec	;				// 0 - 10  ==> 1/4 - 9/10
//	uchar	unused3 ;
//	uchar	output_format ;		// 9       ==> SYMBOLS (1 byte each)
//	uint16	bytes_count ;		// the number of bytes in the BBframe
//  uchar   data[] ;            // *** the pointer returned always points to the start of the BB frame ***
//
//	p = dvbs2neon_packet (...) ;
//	if ((int)p > 0)	 // frame ready
//	{
//		p16 			= (uint16*) p ;
//		symbol_count 	= p16 [-1] ;		
//
//		p8 = (uchar*) p ;
//		output_format	= p8 [-3] ;
//		fec				= p8 [-5]
//		constellation	= p8 [-6]
//		frame_type		= p8 [-7]
//	} 
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
// int32 dvbsneon_control (STREAM,SET_OUTPUT_BUFFER,BUFFER_ADDRESS,0) ;
//
//	Instead of passing a buffer pool address which can hold 4 output buffers which 
//	are used in sequence, an individual buffer address can be set up for each stream.
//
//	This can be changed at any time without loss of data.
//

//	Use an address of zero to go back to using the buffer pool.
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//	dvbs2neon_packet
//
//	Adds a packet to the datafield and returns a pointer to an IQ buffer when a frame is ready 
//	
//	Calling from C:
//		char* dvbs2neon_packet (uint32 stream, uint32 (uint32)&packet188, uint32 param1) ;
//
//		param1 is reserved
//
//	Accepts the address of a 188 byte packet starting with 0x47 (not checked)
//
//	Returns a status indication or a pointer to an output buffer
//		0 = more packets are required to fill the frame
//	   <0 = error
//	   >0 = pointer to a buffer containing a frame, in the selected output format  
//			the buffer may be 32 or 16 bit aligned
//		If the buffer pool is being used, 4 outbuffers are used in sequence, so that 
//		  it is not neccessary to move the data to prevent it being overwritten by 
//		  the next frame	
//
//	All registers except r0 are restored to their original values
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//	Bit fields for normal frame FEC 3/4 QPSK no pilots
//
//					   *********					DATAFIELD	48328
//			  ******************					BBFRAME		48408  
//			  *************************				BCHFRAME	48600
//			  **********************************	FECFRAME	64800
//	********************************************	PLFRAME		64980
//	PLHEADER  BBHEADER DATAFIELD	BCH		LDPC
//	180		  80	   48328		192		16200
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
// Definitions used for external control
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

#define CONTROL_RESET_FULL				0
#define CONTROL_RESET_STREAM			1							
#define CONTROL_SET_PARAMETERS			2							
#define CONTROL_SET_OUTPUT_BUFFER		3							
#define CONTROL_GET_EFFICIENCY			4
#define CONTROL_GET_INFO				5
#define CONTROL_GET_LAST_BBFRAME		6
#define CONTROL_CONVERT_BBFRAME         7
#define CONTROL_GET_DATAFIELD_SIZE      8
#define CONTROL_GET_SYMBOLS_PER_FRAME   9
#define CONTROL_GET_DUMMY_FRAME         10

#define DATAMODE_TS						0		// 4th parameter of RESET_STREAM						
#define DATAMODE_GSECONTINUOUS			1							
#define MULTISTREAM_OFF					0

#define OUTPUT_FORMAT_TS_SYMBOLS 		OUTPUT_FORMAT_SYMBOLS
#define OUTPUT_FORMAT_GSE_SYMBOLS 		OUTPUT_FORMAT_GSESYMBOLS

#define OUTPUT_FORMAT_SYMBOLS							0
#define OUTPUT_FORMAT_IQ								1		// removed
#define OUTPUT_FORMAT_EXPRESS							2		// not yet implemented
#define OUTPUT_FORMAT_BBFRAME							9

#define ERROR_COMMAND_INVALID							-1
#define ERROR_STREAM_INVALID							-2
#define ERROR_FRAME_TYPE_INVALID						-3
#define ERROR_FEC_INVALID								-4
#define ERROR_ROLLOFF_INVALID							-5
#define ERROR_CONSTELLATION_INVALID						-6
#define ERROR_PILOTS_INVALID							-7
#define ERROR_OUTPUT_FORMAT_INVALID						-8
#define ERROR_BUFFER_ADDRESS_INVALID					-9
#define ERROR_FEC_INVALID_FOR_CONSTELLATION				-10
#define ERROR_BUFFER_POOL_INVALID						-11
#define ERROR_NO_OUTBUFFER_AVAILABLE					-12
#define ERROR_OUTPUT_FORMAT_INVALID_FOR_CONSTELLATION	-13
#define ERROR_OUTPUT_FORMAT_INVALID_FOR_ROLLOFF			-14
#define ERROR_FRAME_FEC_COMBINATION_INVALID				-15
#define ERROR_BBFRAME_NOT_AVAILABLE                     -16
#define ERROR_PACKET_SYNC_BYTE_MISSING                  -17
#define ERROR_NOT_IN_TS_MODE				            -18
#define ERROR_NOT_IN_GSE_MODE				            -19
#define ERROR_GSE_DATA_TOO_BIG				            -20
#define ERROR_UNKNOWN						            -21
#define ERROR_INVALID_MULTISTREAM						-22
#define ERROR_INVALID_DATAMODE							-23
#define ERROR_LOWEST						            -23



	

// parameter definitions for DATV-Express compatibility																				

#define CR_1_4				0
#define CR_1_3				1
#define CR_2_5				2
#define CR_1_2				3
#define CR_3_5				4
#define CR_2_3				5
#define CR_3_4				6
#define CR_4_5				7
#define CR_5_6				8
#define CR_8_9				9
#define CR_9_10				10
#define M_QPSK				0
#define M_8PSK				1
#define M_16APSK			2
#define M_32APSK			3
#define FRAME_NORMAL		0x00
#define FRAME_SHORT			0x10
#define PILOTS_OFF			0
#define PILOTS_ON			1
#define RO_0_35				0
#define RO_0_25				1
#define RO_0_20				2
#define RO_0_15				3

#define FEC14							0
#define FEC13							1
#define FEC25							2
#define FEC12							3
#define FEC35							4
#define FEC23							5
#define FEC34							6
#define FEC45							7
#define FEC56							8
#define FEC89							9
#define FEC910							10

#define DVBS_MODE						1
#define DVBS2_MODE						2
#define FRAME_TYPE_NORMAL				0
#define FRAME_TYPE_SHORT				2
#define ROLLOFF_035						0
#define ROLLOFF_025						1
#define ROLLOFF_020						2
#define ROLLOFF_015						3
#define PILOTS_OFF						0
#define PILOTS_ON						1
#define QPSK							0
#define PSK8							1
#define APSK16							2
#define APSK32							3

//#define	SYMBOLS_METADATA_OFFSET			10 			// symbols start 10 bytes into the output buffer
//#define BBFRAME_METADATA_OFFSET         12          // BB frame start 10 bytes into the output buffer (get_last_bbframe)
//#define	FILTER_OFFSET					8 			// filtered values start 10 bytes into the output buffer

#define	SYMBOLS_METADATA_OFFSET			38 			@: symbols start 38 bytes into the output buffer
#define BBFRAME_METADATA_OFFSET         40          @: BB frame data starts 40 bytes into the output buffer 
#define DUMMY_FRAME_METADATA_OFFSET     38          @: dummy frame data starts 38 bytes into the output buffer



#define STREAM0		0


typedef struct {
	short re;
	short im;
}sfcmplx;


sfcmplx symbol_lut[32];
//#define CP 0x7FFF
#define CP 0x7FFF

void modulator_mapping(int constel,int CodeRate,float diggain=1.0,int dpdrotate=0,float dpdmag=1.0,int dpdrotate1=0,float dpdmag1=1.0,int dpdrotate2=0,float dpdmag2=1.0,float dpdmag3=1.0)
{
        double r0,r1,r2,r3;
        double m = diggain;
        r0 = 1.0;// I am not sure why this needs to be 0.9 but 32APSK does not work if == 1.0
        r1 = m;
        double fdpdrotate=M_PI*((double)dpdrotate/180.0f);
        double fdpdrotate1=M_PI*((double)dpdrotate1/180.0f);
        double fdpdrotate2=M_PI*((double)dpdrotate2/180.0f);
    switch(constel)
    {
        case M_QPSK:
        // QPSK
        symbol_lut[0].re = (short)((r1*cos(M_PI/4.0))*CP);
        symbol_lut[0].im = (short)((r1*sin(M_PI/4.0))*CP);
        symbol_lut[1].re = (short)((r1*cos(7*M_PI/4.0))*CP);
        symbol_lut[1].im = (short)((r1*sin(7*M_PI/4.0))*CP);
        symbol_lut[2].re = (short)((r1*cos(3*M_PI/4.0))*CP);
        symbol_lut[2].im = (short)((r1*sin(3*M_PI/4.0))*CP);
        symbol_lut[3].re = (short)((r1*cos(5*M_PI/4.0))*CP);
        symbol_lut[3].im = (short)((r1*sin(5*M_PI/4.0))*CP);
        break;

        case M_8PSK:
	// 8PSK
        symbol_lut[0].re = (short)((r1*cos(M_PI/4.0))*CP);
        symbol_lut[0].im = (short)((r1*sin(M_PI/4.0))*CP);
        symbol_lut[1].re = (short)((r1*cos(0.0))*CP);
        symbol_lut[1].im = (short)((r1*sin(0.0))*CP);
        symbol_lut[2].re = (short)((r1*cos(4*M_PI/4.0))*CP);
        symbol_lut[2].im = (short)((r1*sin(4*M_PI/4.0))*CP);
        symbol_lut[3].re = (short)((r1*cos(5*M_PI/4.0))*CP);
        symbol_lut[3].im = (short)((r1*sin(5*M_PI/4.0))*CP);
        symbol_lut[4].re = (short)((r1*cos(2*M_PI/4.0))*CP);
        symbol_lut[4].im = (short)((r1*sin(2*M_PI/4.0))*CP);
        symbol_lut[5].re = (short)((r1*cos(7*M_PI/4.0))*CP);
        symbol_lut[5].im = (short)((r1*sin(7*M_PI/4.0))*CP);
        symbol_lut[6].re = (short)((r1*cos(3*M_PI/4.0))*CP);
        symbol_lut[6].im = (short)((r1*sin(3*M_PI/4.0))*CP);
        symbol_lut[7].re = (short)((r1*cos(6*M_PI/4.0))*CP);
        symbol_lut[7].im = (short)((r1*sin(6*M_PI/4.0))*CP);
        break;

        case M_16APSK:
        // 16 APSK
        
        r2 = m;
        switch( CodeRate )
        {
            case CR_2_3:
                r1 = r2/3.15;
                break;
            case CR_3_4:
                r1 = r2/2.85;
                break;
            case CR_4_5:
                r1 = r2/2.75;
                break;
            case CR_5_6:
                r1 = r2/2.70;
                break;
            case CR_8_9:
                r1 = r2/2.60;
                break;
            case CR_9_10:
                r1 = r2/2.57;
                break;
            default:
                // Illegal
                r1 = 0;
                break;
        }
        r1=r1*dpdmag; // We correct magnitude of centre symbols
        symbol_lut[0].re  = (short)((r2*cos(M_PI/4.0))*CP);
        symbol_lut[0].im  = (short)((r2*sin(M_PI/4.0))*CP);
        symbol_lut[1].re  = (short)((r2*cos(-M_PI/4.0))*CP);
        symbol_lut[1].im  = (short)((r2*sin(-M_PI/4.0))*CP);
        symbol_lut[2].re  = (short)((r2*cos(3*M_PI/4.0))*CP);
        symbol_lut[2].im  = (short)((r2*sin(3*M_PI/4.0))*CP);
        symbol_lut[3].re  = (short)((r2*cos(-3*M_PI/4.0))*CP);
        symbol_lut[3].im  = (short)((r2*sin(-3*M_PI/4.0))*CP);
        symbol_lut[4].re  = (short)((r2*cos(M_PI/12.0))*CP);
        symbol_lut[4].im  = (short)((r2*sin(M_PI/12.0))*CP);
        symbol_lut[5].re  = (short)((r2*cos(-M_PI/12.0))*CP);
        symbol_lut[5].im  = (short)((r2*sin(-M_PI/12.0))*CP);
        symbol_lut[6].re  = (short)((r2*cos(11*M_PI/12.0))*CP);
        symbol_lut[6].im  = (short)((r2*sin(11*M_PI/12.0))*CP);
        symbol_lut[7].re  = (short)((r2*cos(-11*M_PI/12.0))*CP);
        symbol_lut[7].im  = (short)((r2*sin(-11*M_PI/12.0))*CP);
        symbol_lut[8].re  = (short)((r2*cos(5*M_PI/12.0))*CP);
        symbol_lut[8].im  = (short)((r2*sin(5*M_PI/12.0))*CP);
        symbol_lut[9].re  = (short)((r2*cos(-5*M_PI/12.0))*CP);
        symbol_lut[9].im  = (short)((r2*sin(-5*M_PI/12.0))*CP);
        symbol_lut[10].re = (short)((r2*cos(7*M_PI/12.0))*CP);
        symbol_lut[10].im = (short)((r2*sin(7*M_PI/12.0))*CP);
        symbol_lut[11].re = (short)((r2*cos(-7*M_PI/12.0))*CP);
        symbol_lut[11].im = (short)((r2*sin(-7*M_PI/12.0))*CP);
        symbol_lut[12].re = (short)((r1*cos(M_PI/4.0+fdpdrotate))*CP);
        symbol_lut[12].im = (short)((r1*sin(M_PI/4.0+fdpdrotate))*CP);
        symbol_lut[13].re = (short)((r1*cos(-M_PI/4.0+fdpdrotate))*CP);
        symbol_lut[13].im = (short)((r1*sin(-M_PI/4.0+fdpdrotate))*CP);
        symbol_lut[14].re = (short)((r1*cos(3*M_PI/4.0+fdpdrotate))*CP);
        symbol_lut[14].im = (short)((r1*sin(3*M_PI/4.0+fdpdrotate))*CP);
        symbol_lut[15].re = (short)((r1*cos(-3*M_PI/4.0+fdpdrotate))*CP);
        symbol_lut[15].im = (short)((r1*sin(-3*M_PI/4.0+fdpdrotate))*CP);

        break;
        case M_32APSK:
        // 32 APSK
        //r3 = 0.8;
        r3=1.0;
        //r2=0.9;
		switch( CodeRate )
        {
            case CR_3_4:
                r1 = r3/5.27;
                r2 = r1*2.84;
                break;
            case CR_4_5:
                r1 = r3/4.87;
                r2 = r1*2.72;
                break;
            case CR_5_6:
                r1 = r3/4.64;
                r2 = r1*2.64;
                break;
            case CR_8_9:
                r1 = r3/4.33;
                r2 = r1*2.54;
                break;
            case CR_9_10:
                r1 = r3/4.30;
                r2 = r1*2.53;
                break;
            default:
                // Illegal
                r1 = 0;
                r2 = 0;
                break;
        }
	r1=r1*dpdmag1;
	r2=r2*dpdmag2;
	r3=r3*dpdmag3;
	symbol_lut[0].re  = (short)((r2*cos(M_PI/4.0+fdpdrotate2))*CP);
        symbol_lut[0].im  = (short)((r2*sin(M_PI/4.0+fdpdrotate2))*CP);
        symbol_lut[1].re  = (short)((r2*cos(5*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[1].im  = (short)((r2*sin(5*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[2].re  = (short)((r2*cos(-M_PI/4.0+fdpdrotate2))*CP);
        symbol_lut[2].im  = (short)((r2*sin(-M_PI/4.0+fdpdrotate2))*CP);
        symbol_lut[3].re  = (short)((r2*cos(-5*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[3].im  = (short)((r2*sin(-5*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[4].re  = (short)((r2*cos(3*M_PI/4.0+fdpdrotate2))*CP);
        symbol_lut[4].im  = (short)((r2*sin(3*M_PI/4.0+fdpdrotate2))*CP);
        symbol_lut[5].re  = (short)((r2*cos(7*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[5].im  = (short)((r2*sin(7*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[6].re  = (short)((r2*cos(-3*M_PI/4.0+fdpdrotate2))*CP);
        symbol_lut[6].im  = (short)((r2*sin(-3*M_PI/4.0+fdpdrotate2))*CP);
        symbol_lut[7].re  = (short)((r2*cos(-7*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[7].im  = (short)((r2*sin(-7*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[8].re  = (short)((r3*cos(M_PI/8.0))*CP);
        symbol_lut[8].im  = (short)((r3*sin(M_PI/8.0))*CP);
        symbol_lut[9].re  = (short)((r3*cos(3*M_PI/8.0))*CP);
        symbol_lut[9].im  = (short)((r3*sin(3*M_PI/8.0))*CP);
        symbol_lut[10].re = (short)((r3*cos(-M_PI/4.0))*CP);
        symbol_lut[10].im = (short)((r3*sin(-M_PI/4.0))*CP);
        symbol_lut[11].re = (short)((r3*cos(-M_PI/2.0))*CP);
        symbol_lut[11].im = (short)((r3*sin(-M_PI/2.0))*CP);
        symbol_lut[12].re = (short)((r3*cos(3*M_PI/4.0))*CP);
        symbol_lut[12].im = (short)((r3*sin(3*M_PI/4.0))*CP);
        symbol_lut[13].re = (short)((r3*cos(M_PI/2.0))*CP);
        symbol_lut[13].im = (short)((r3*sin(M_PI/2.0))*CP);
        symbol_lut[14].re = (short)((r3*cos(-7*M_PI/8.0))*CP);
        symbol_lut[14].im = (short)((r3*sin(-7*M_PI/8.0))*CP);
        symbol_lut[15].re = (short)((r3*cos(-5*M_PI/8.0))*CP);
        symbol_lut[15].im = (short)((r3*sin(-5*M_PI/8.0))*CP);
        symbol_lut[16].re = (short)((r2*cos(M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[16].im = (short)((r2*sin(M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[17].re = (short)((r1*cos(M_PI/4.0+fdpdrotate1))*CP);
        symbol_lut[17].im = (short)((r1*sin(M_PI/4.0+fdpdrotate1))*CP);
        symbol_lut[18].re = (short)((r2*cos(-M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[18].im = (short)((r2*sin(-M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[19].re = (short)((r1*cos(-M_PI/4.0+fdpdrotate1))*CP);
        symbol_lut[19].im = (short)((r1*sin(-M_PI/4.0+fdpdrotate1))*CP);
        symbol_lut[20].re = (short)((r2*cos(11*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[20].im = (short)((r2*sin(11*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[21].re = (short)((r1*cos(3*M_PI/4.0+fdpdrotate1))*CP);
        symbol_lut[21].im = (short)((r1*sin(3*M_PI/4.0+fdpdrotate1))*CP);
        symbol_lut[22].re = (short)((r2*cos(-11*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[22].im = (short)((r2*sin(-11*M_PI/12.0+fdpdrotate2))*CP);
        symbol_lut[23].re = (short)((r1*cos(-3*M_PI/4.0+fdpdrotate1))*CP);
        symbol_lut[23].im = (short)((r1*sin(-3*M_PI/4.0+fdpdrotate1))*CP);
        symbol_lut[24].re = (short)((r3*cos(0.0))*CP);
        symbol_lut[24].im = (short)((r3*sin(0.0))*CP);
        symbol_lut[25].re = (short)((r3*cos(M_PI/4.0))*CP);
        symbol_lut[25].im = (short)((r3*sin(M_PI/4.0))*CP);
        symbol_lut[26].re = (short)((r3*cos(-M_PI/8.0))*CP);
        symbol_lut[26].im = (short)((r3*sin(-M_PI/8.0))*CP);
        symbol_lut[27].re = (short)((r3*cos(-3*M_PI/8.0))*CP);
        symbol_lut[27].im = (short)((r3*sin(-3*M_PI/8.0))*CP);
        symbol_lut[28].re = (short)((r3*cos(7*M_PI/8.0))*CP);
        symbol_lut[28].im = (short)((r3*sin(7*M_PI/8.0))*CP);
        symbol_lut[29].re = (short)((r3*cos(5*M_PI/8.0))*CP);
        symbol_lut[29].im = (short)((r3*sin(5*M_PI/8.0))*CP);
        symbol_lut[30].re = (short)((r3*cos(M_PI))*CP);
        symbol_lut[30].im = (short)((r3*sin(M_PI))*CP);
        symbol_lut[31].re = (short)((r3*cos(-3*M_PI/4.0))*CP);
        symbol_lut[31].im = (short)((r3*sin(-3*M_PI/4.0))*CP);
        break;
    }
    /*
    for(int i=0;i<32;i++)
    {
        fprintf(stderr,"SymbolLUT(%d)=%d + i %d mag %d\n",i,symbol_lut[i].re,symbol_lut[i].im,symbol_lut[i].re*symbol_lut[i].re+symbol_lut[i].im*symbol_lut[i].im);
    }
    */
}
#endif