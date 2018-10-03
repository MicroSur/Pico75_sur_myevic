#include "myevic.h"
#include "display.h"
#include "dataflash.h"
#include "screens.h"


//=========================================================================

uint8_t	DisplayModel;
uint8_t	DisplayCmdByte;

const image_t **Images;

uint8_t ScreenBuffer[SCREEN_BUFFER_SIZE] __attribute__((aligned(8)));
uint8_t ScreenBuffer_96_16[SCREEN_BUFFER_SIZE_96_16] __attribute__((aligned(8)));

const uint8_t ByteMaskRight[] = { 0x00, 0x01, 0x03,	0x07, 0x0F, 0x1F, 0x3F,	0x7F };
const uint8_t ByteMaskLeft[]  = { 0xFF, 0xFE, 0xFC,	0xF8, 0xF0, 0xE0, 0xC0,	0x80 };


//=========================================================================

typedef void (PLOT_FUNC(int,int,int));

PLOT_FUNC *DrawPoint;

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables  i2c                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr = 0x3C;
uint8_t g_au8TxData[97];
//volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataIndex;
volatile uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);
static volatile I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
__myevic__ void I2C_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                      //START has been transmitted
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);    // Write SLA+W to Register I2CDAT
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  // SLA+W has been transmitted and ACK has been received
    {
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataIndex++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  // SLA+W has been transmitted and NACK has been received 
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  // DATA has been transmitted and ACK has been received
    {
        if(g_u8DataIndex != g_u8DataLen )
        {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataIndex++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8EndFlag = 1;
        }
    }
}

__myevic__ void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);
    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        // Clear I2C0 Timeout Flag
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
    
}


//=========================================================================
//----- (00006228) --------------------------------------------------------
__myevic__ void InitSPI0()
{
	SYS->GPE_MFPH &= (~(SYS_GPE_MFPH_PE11MFP_Msk|SYS_GPE_MFPH_PE12MFP_Msk|SYS_GPE_MFPH_PE13MFP_Msk));
	SYS->GPE_MFPH |= (SYS_GPE_MFPH_PE11MFP_SPI0_MOSI0|SYS_GPE_MFPH_PE12MFP_SPI0_SS|SYS_GPE_MFPH_PE13MFP_SPI0_CLK);

	SPI_Open( SPI0, SPI_MASTER, SPI_MODE_0, 8, 4000000 );
	SPI_EnableAutoSS( SPI0, SPI_SS, SPI_SS_ACTIVE_LOW );
}

__myevic__ void InitI2C()
{
        //SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk|SYS_GPA_MFPL_PA3MFP_Msk);
        //SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_I2C0_SDA|SYS_GPA_MFPL_PA3MFP_I2C0_SCL); //0x400 0x4000 = 0x4400
 
    // Open I2C module and set bus clock
    I2C_Open( I2C0, 100000 );
    
    // Set I2C 4 Slave Addresses
    I2C_SetSlaveAddr(I2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, 0);   /* Slave Address : 0x75 */

    // Enable I2C interrupt
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);

}


//=========================================================================
//----- (00005160) --------------------------------------------------------
__myevic__ void DisplaySendCommand( const uint8_t cmd )
{
	DisplayCmdByte = cmd;

	switch ( DisplayModel )
	{
		case 0:                    
			SSD1306_WriteBytes( 0, &DisplayCmdByte, 1 );
			break;
			
		case 1:
			SSD1327_WriteBytes( 0, &DisplayCmdByte, 1 );
			break;
                        
                case 3:                    
			SSD1306_96_16_WriteBytes( 0, &DisplayCmdByte, 1 );
			break;                        
	}
}


//=========================================================================
//----- (00005184) --------------------------------------------------------
__myevic__ void DisplaySendData( const uint8_t *data, const uint32_t len )
{
	switch ( DisplayModel )
	{
		case 0:  
			SSD1306_WriteBytes( 0x40, data, len );
			break;

		case 1:
			SSD1327_WriteBytes( 0x40, data, len );
			break;

                case 3:   
			SSD1306_96_16_WriteBytes( 0x40, data, len );
			break;                        
	}
}


//=========================================================================
//----- (00005240) --------------------------------------------------------
__myevic__ void InitDisplay()
{
	switch ( DisplayModel )
	{
		case 0:
			SSD1306_Init();
			DrawPoint = SSD1306_Plot+1;
			break;

		case 1:
			SSD1327_Init();
			DrawPoint = SSD1327_Plot+1;
			break;
                        
                case 3:
                    	SSD1306_96_16_Init();
			DrawPoint = SSD1306_96_16_Plot+1;
			break;
	}
        

        if ( gFlags.MainContrast )
            DisplaySetContrast( dfContrast );
        else
            DisplaySetContrast( dfContrast2 );
        
	DisplaySetInverse( dfStatus.invert );        

        Images = font0_1306; //DisplaySetFont();        

}

//=========================================================================
__myevic__ void DisplaySetContrast( const uint8_t c )
{
	switch ( DisplayModel )
	{
		case 0:
                case 3:
			SSD1306_SetContrast( c );
			break;

		case 1:
			SSD1327_SetContrast( c );
			break;
	}
}

//=========================================================================
__myevic__ void DisplaySetInverse( const uint8_t i )
{
	switch ( DisplayModel )
	{
		case 0:
                case 3:    
			SSD1306_SetInverse( i );
			break;

		case 1:
			SSD1327_SetInverse( i );
			break;
	}
}

//=========================================================================

/*
__myevic__ void DrawTimeSmall( int x, int y, S_RTC_TIME_DATA_T *rtd, int colors )
{
	if ( gFlags.draw_edited_item ) colors = 0x1F;

	if (colors&0x10) DrawValue( x   , y, rtd->u32Hour, 0, 0x0B, 2 );
	if (colors&0x08) DrawImage( x+12, y, 0xD7 );
	if (colors&0x04) DrawValue( x+15, y, rtd->u32Minute, 0, 0x0B, 2 );
	if (colors&0x02) DrawImage( x+27, y, 0xD7 );
	if (colors&0x01) DrawValue( x+30, y, rtd->u32Second, 0, 0x0B, 2 );
}
*/

__myevic__ void DrawTime( int x, int y, S_RTC_TIME_DATA_T *rtd, int colors )
{
    //24h only
	if ( gFlags.draw_edited_item ) colors = 0x1F;

	if (colors&0x10) DrawValue( x   , y, rtd->u32Hour, 0, 0x1F, 2 );
	if (colors&0x08) DrawImage( x+17, y+2, 0xDD );
	if (colors&0x04) DrawValue( x+21, y, rtd->u32Minute, 0, 0x1F, 2 );
	if (colors&0x02) DrawImage( x+38, y+2, 0xDD );
	if (colors&0x01) DrawValue( x+42, y, rtd->u32Second, 0, 0x1F, 2 );
}

typedef struct
{
	uint8_t separator;
	uint8_t	dayoffset;
	uint8_t	monthoffset;
	uint8_t	yearoffset;
	uint8_t sep1offset;
	uint8_t sep2offset;
}
datefmt_t;

__myevic__ uint8_t dayofweek(uint32_t d, uint32_t m, uint32_t y) {
    static int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    y -= m < 3;
    return ( y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

__myevic__ void DrawDate( int x, int y, S_RTC_TIME_DATA_T *rtd, int colors )
{ 
    	//uint8_t separator;
	//uint8_t	dayoffset;
	//uint8_t	monthoffset;
	//uint8_t	yearoffset;
	//uint8_t sep1offset;
	//uint8_t sep2offset;
        
	const datefmt_t format[] =
	{
		{ 0xC1,  0, 16, 32, 13, 29 },
		{ 0xD6, 16,  0, 32, 12, 28 },
		{ 0xD6,  0, 16, 32, 12, 28 },
		{ 0xD9, 44, 28,  0, 24, 40 }
	};

	const datefmt_t *f = &format[ dfStatus.dfmt1 | ( dfStatus.dfmt2 << 1 ) ];

	if ( gFlags.draw_edited_item ) colors = 0x1F;

	if (colors&0x10) DrawValue( x + f->dayoffset,   y, rtd->u32Day,   0, 0x0B, 2 );
	if (colors&0x04) DrawValue( x + f->monthoffset, y, rtd->u32Month, 0, 0x0B, 2 );
	if (colors&0x01) DrawValue( x + f->yearoffset,  y, rtd->u32Year,  0, 0x0B, 4 );
	if (colors&0x08) DrawImage( x + f->sep1offset,  y, f->separator );
	if (colors&0x02) DrawImage( x + f->sep2offset,  y, f->separator );
        
        y += 13;
            //switch ( rtd->u32DayOfWeek )
        
        const uint8_t *WEEKS[] = { String_Sunday, String_Monday, String_Tuesday, 
                            String_Wednesday, String_Thursday, String_Friday, 
                            String_Saturday};
        DrawStringCentered( WEEKS[dayofweek ( rtd->u32Day, rtd->u32Month, rtd->u32Year )], y );
}


//=========================================================================
//----- (00005784) --------------------------------------------------------
__myevic__ void ClearScreenBuffer()
{
	switch ( DisplayModel )
	{
		case 0:    
			SSD1306_ClearBuffer();
			break;

		case 1:
			SSD1327_ClearBuffer();
			break;
                        
                case 3:       
        		SSD1306_96_16_ClearBuffer();
			break;
	}
}


//=========================================================================
//----- (00005AAC) --------------------------------------------------------
__myevic__ void ScreenOff()
{
	switch ( DisplayModel )
	{
		case 0:
                case 3:
			SSD1306_ScreenOff();
			break;

		case 1:
			SSD1327_ScreenOff();
			break;
	}
}


//=========================================================================
//----- (00005AC4) --------------------------------------------------------
__myevic__ void DisplayRefresh()
{
	switch ( DisplayModel )
	{
		case 0:
			SSD1306_Refresh();
			break;

		case 1:
			SSD1327_Refresh();
			break;
                        
		case 3:
			SSD1306_96_16_Refresh();
			break;                        
	}
}


//=========================================================================
//----- (000051CC) --------------------------------------------------------
__myevic__ int GetImageWidth( const uint8_t imgnum )
{
	return Images[imgnum - 1]->width;
}


//=========================================================================
__myevic__ int GetStringWidth( const uint8_t str[] )
{
	int width = 0;

	for ( int l = 0 ; str[l] ; ++l )
	{
		width += GetImageWidth( str[l] );
	}

	return width;
}


//=========================================================================
//----- (000051A8) --------------------------------------------------------
__myevic__ int GetStrCenteredX( const uint8_t str[] )
{
	return ( ( 64 - GetStringWidth( str ) ) >> 1 );
}


//=========================================================================
//----- (000051F8) --------------------------------------------------------
__myevic__ void DrawHLine( const int x1, const int y, const int x2, const int color )
{
	int inc = ( x1 < x2 ) * 2 - 1;

	for ( int x = x1 ; x != x2 + inc ; x += inc )
	{
		DrawPoint( x, y, color );
	}
}

__myevic__ void DrawHLineDots( const int x1, const int y, const int x2, const int color )
{
	int inc = ( x1 < x2 ) * 2 - 1;

	for ( int x = x1 ; x <= x2 + 2*inc ; x += 2 * inc )
	{
		DrawPoint( x, y, color );
	}
}

//=========================================================================
__myevic__ void DrawVLine( const int x, const int y1, const int y2, const int color )
{
	int inc = ( y1 < y2 ) * 2 - 1;

	for ( int y = y1 ; y != y2 + inc ; y += inc )
	{
		DrawPoint( x, y, color );
	}
}

//=========================================================================
__myevic__ void DrawVLineDots( const int x, const int y1, const int y2 )
{
	int inc = ( y1 < y2 ) * 2 - 1;

	for ( int y = y1 ; y < y2 + 2*inc ; y += 2*inc )
	{
		DrawPoint( x, y, 1 );
	}
}

//=========================================================================
//----- (0000575C) --------------------------------------------------------
__myevic__ void DrawFillRect( const int x1, const int y1,const  int x2, const int y2, const int color)
{
	for ( int y = y1 ; y <= y2 ; ++y )
	{
		DrawHLine( x1, y, x2, color );
	}
}
__myevic__ void DrawFillRectLines( const int x1, const int y1,const  int x2, const int y2, const int color)
{
	for ( int y = y1 ; y <= y2 ; ++y )
	{
		DrawHLineDots( x1, y, x2, color );
	}
}

/*
__myevic__ void DrawRect( const int x1, const int y1,const  int x2, const int y2, const int color)
{
    	DrawHLine( x1, y1,  x2, color );
	DrawHLine( x1, y2,  x2, color );

	DrawVLine( x1, y1, y2, color );
	DrawVLine( x2, y1, y2, color );
}
*/

//=========================================================================
//----- (0000579C) --------------------------------------------------------
__myevic__ uint32_t DrawImage( const int x, const int y, const uint8_t img )
{
	switch ( DisplayModel )
	{
		case 0:    
			return SSD1306_Image( x, y, img, 0 );

		case 1:
			return SSD1327_Image( x, y, img, 0 );
                        
                case 3:    
			return SSD1306_96_16_Image( x, y, img, 0 );
                        
		default:
			return 0;
	}
}

__myevic__ uint32_t DrawImageRight( const int x, const int y, const uint8_t img )
{
	switch ( DisplayModel )
	{
		case 0:    
			return SSD1306_Image( x - Images[img - 1]->width, y, img, 0 );

		case 1:
			return SSD1327_Image( x - Images[img - 1]->width, y, img, 0 );
                        
                case 3:    
			return SSD1306_96_16_Image( x - Images[img - 1]->width, y, img, 0 );

		default:
			return 0;
	}
}

/*
__myevic__ void DrawImgImgImg( const int x, const int y, const uint8_t img1, const uint8_t img2, const uint8_t img3 )
{
	switch ( DisplayModel )
	{
		case 0:
			SSD1306_Image( x, y, img1, 0 );
                        SSD1306_Image( x + Images[img1 - 1]->width, y, img2, 0 );
                        SSD1306_Image( x + Images[img2 - 1]->width, y, img3, 0 );


		case 1:
                        SSD1327_Image( x, y, img1, 0 );
                        SSD1327_Image( x + Images[img1 - 1]->width, y, img2, 0 );
                        SSD1327_Image( x + Images[img2 - 1]->width, y, img3, 0 );
	}
}
*/

//=========================================================================
//----- (000057B8) --------------------------------------------------------
__myevic__ uint32_t DrawImageInv( const int x, const int y, const uint8_t img )
{
	switch ( DisplayModel )
	{
		case 0:    
			return SSD1306_Image( x, y, img, 1 );

		case 1:
			return SSD1327_Image( x, y, img, 1 );
                        
                case 3:    
			return SSD1306_96_16_Image( x, y, img, 1 );
                        
		default:
			return 0;
	}
}



//=========================================================================
// Retrieve the Logo height (0=No logo)
//-------------------------------------------------------------------------
__myevic__ int GetLogoHeight()
{
	image_t *img;
	int h;

	img = (image_t*)DATAFLASH_LOGO_1306_BASE;

	//h = 0;

	//if ( img->width == 64 )
	//{
		h = img->height;
		//if ( h < 40 || h > 63 ) h = 0;
	//}

	return h;
}


//=========================================================================
__myevic__ void DrawLOGO( const int x, const int y )
{
	image_t *img;

	if (( dfStatus.nologo ) && ( Screen != 60 )) return;

	img = (image_t*)DATAFLASH_LOGO_1306_BASE;

	//if ( img->width == 64 && img->height >= 40 && img->height <= 63 )
	//{
		switch ( DisplayModel )
		{
			case 0:    
				SSD1306_Bitmap( x, y, img, 0 );
				break;

			case 1:
				SSD1327_Bitmap( x, y, img, 0 );
				break;     
                                
                        case 3:       
                                SSD1306_96_16_Bitmap( x, y, img, 0 );
				break;
		}
	//}
}


//=========================================================================
//-------------------------------------------------------------------------
__myevic__ uint8_t* Value2Str( uint8_t *str, int v, int dp, uint8_t z, int nd )
{
	uint8_t dot;
	int i, d;

	if ( !nd )
	{
		for ( int i = v ; i ; i /= 10 ) ++nd;
		if ( nd <= dp ) nd = dp + 1;
	}

	for ( i = 0 ; i < nd ; ++i )
	{
		d = v % 10;
		v /= 10;
		str[nd - i - 1] = d + z;
	}
	str[i] = 0;

	if ( dp && dp <= nd )
	{
		int dot_pos = nd - dp;

		while ( nd >= dot_pos )
		{
			str[nd + 1] = str[nd];
			--nd;
		}

		switch ( z )
		{
			default:
			case 0x0B:
				dot = 0xC1;
				break;
			//case 0x15:
			//	dot = 0xD8;
			//	break;
			case 0x1F:
				dot = 0xDC;
				break;
			case 0x29:
				dot = 0xDF;
				break;
			//case 0x33:
			//	dot = 0xF1;
			//	break;
			case 0x3D:
				dot = 0x47;
				break;
			case 0x48:
				dot = 0xF7;
				break;
			//case 0x52:
			//	dot = 0xF9;
			//	break;
			//case 0x5C:
			//	dot = 0x66;
			//	break;
		}

		str[nd + 1] = dot;
	}
	return str;
}


//=========================================================================
//----- (000058A4) --------------------------------------------------------
__myevic__ void DrawValue( int x, int y, int v, uint8_t dp, uint8_t z, uint8_t nd )
{
	uint8_t str[12];

	Value2Str( str, v, dp, z, nd );
	DrawString( str, x, y );
}


//=========================================================================
// Draw right-justified numerical value
//-------------------------------------------------------------------------
__myevic__ void DrawValueRight( int x, int y, int v, uint8_t dp, uint8_t z, uint8_t nd )
{
	uint8_t str[12];

	Value2Str( str, v, dp, z, nd );
	DrawString( str, x - GetStringWidth( str ), y );
}


//=========================================================================
//----- (000058A4) --------------------------------------------------------
__myevic__ void DrawValueInv( int x, int y, int v, uint8_t dp, uint8_t z, uint8_t nd )
{
	uint8_t str[12];

	Value2Str( str, v, dp, z, nd );
	DrawStringInv( str, x, y );
}


//=========================================================================
//----- (00005A52) --------------------------------------------------------
__myevic__ void DrawString( const uint8_t s[], int x, int y )
{
	while ( *s )
	{
		x += DrawImage( x, y, *s++ );
	}
}


//=========================================================================
//----- (00005A72) --------------------------------------------------------
__myevic__ void DrawStringInv( const uint8_t s[], int x, int y )
{
	while ( *s )
	{
		x += DrawImageInv( x, y, *s++ );
	}
}


//=========================================================================
//----- (00005A92) --------------------------------------------------------
__myevic__ void DrawStringCentered( const uint8_t s[], int y )
{
	DrawString( s, GetStrCenteredX( s ), y );
}


//=========================================================================
__myevic__ void DrawStringRight( const uint8_t s[], int x, int y )
{
	DrawString( s, x - GetStringWidth( s ), y );
}


//=========================================================================
__myevic__ void DrawLine( int x1, int y1, int x2, int y2, int color, int thick )
{
	int dx, dy, e;
	int xinc, yinc;
	int h1, h2;

	if ( !thick ) return;

	h1 = thick >> 1;
	h2 = thick - h1 - 1;

	dx = ( x2 - x1 ) << 1;
	dy = ( y2 - y1 ) << 1;

	if ( dy < 0 )
	{
		yinc = -1;
		dy = -dy;
	}
	else
	{
		yinc = 1;
	}
	
	if ( dx < 0 )
	{
		xinc = -1;
		dx = -dx;
	}
	else
	{
		xinc = 1;
	}
	
	if ( dx > dy )
	{
		e = dx >> 1;
		while ( x1 != x2 )
		{
			DrawVLine( x1, y1 - h1, y1 + h2, color );

			x1 += xinc;
			if (( e -= dy ) <= 0 )
			{
				y1 += yinc;
				e += dx;
			}
		}

		DrawVLine( x2, y2 - h1, y2 + h2, color );
	}
	else
	{
		e = dy >> 1;
		while ( y1 != y2 )
		{
			DrawHLine( x1 - h1, y1, x1 + h2, color );

			y1 += yinc;
			if (( e -= dx ) <= 0 )
			{
				x1 += xinc;
				e += dy;
			}
		}

		DrawHLine( x2 - h1, y2, x2 + h2, color );
	}
}


//=========================================================================
//
// Algorithme de tracé de cercle d'Andres
//
__myevic__ void DrawCircle( int x_centre, int y_centre, int r, int color, int fill )
{
	int x = 0;
	int y = r;
	int d = r - 1;

	while( y >= x )
	{
		if ( fill )
		{
			DrawHLine( x_centre - x, y_centre - y, x_centre + x, color );
			DrawHLine( x_centre - x, y_centre + y, x_centre + x, color );
			DrawHLine( x_centre - y, y_centre - x, x_centre + y, color );
			DrawHLine( x_centre - y, y_centre + x, x_centre + y, color );
		}
		else
		{
			DrawPoint( x_centre - x, y_centre - y, color );
			DrawPoint( x_centre + x, y_centre - y, color );
			DrawPoint( x_centre - x, y_centre + y, color );
			DrawPoint( x_centre + x, y_centre + y, color );
			DrawPoint( x_centre - y, y_centre + x, color );
			DrawPoint( x_centre + y, y_centre + x, color );
			DrawPoint( x_centre - y, y_centre - x, color );
			DrawPoint( x_centre + y, y_centre - x, color );
		}

		if (d >= 2*x)
		{
			d -= 2*x + 1;
			x ++;
		}
		else if (d < 2 * (r-y))
		{
			d += 2*y - 1;
			y --;
		}
		else
		{
			d += 2*(y - x - 1);
			y --;
			x ++;
		}
	}
}


//=========================================================================
// Converts screen buffer in something readable
//-------------------------------------------------------------------------
__myevic__ void Screen2Bitmap( uint8_t *pu8Bitmap )
{
	switch ( DisplayModel )
	{
		case 0:
			SSD1306_Screen2Bitmap( pu8Bitmap );
			break;

		case 1:
			SSD1327_Screen2Bitmap( pu8Bitmap );
			break;
                        
		case 3:
			SSD1306_96_16_Screen2Bitmap( pu8Bitmap );
			break;                        
	}
}


__myevic__ void DrawPixel (int x, int y, int color)
{
	DrawPoint( x, y, color );
}


/*
void I2C0_Close(void)
{
    // Disable I2C0 interrupt and clear corresponding NVIC bit
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    // Disable I2C0 and close I2C0 clock
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}
*/


__myevic__ void SSD1306_96_16_WriteBytes( const int isData, const uint8_t data[], const int len )
{

        g_au8TxData[0] = isData;
        MemCpy( &g_au8TxData[1], data, len );
            
	g_u8DataIndex = 0;
        g_u8EndFlag = 0;
        g_u8DataLen = len+1;
        
        // I2C function to write data to slave
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

        // I2C as master sends START signal
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);
        
	// Wait I2C Tx Finish 
        while(g_u8EndFlag == 0);
   
        g_u8EndFlag = 0;
        
        s_I2C0HandlerFn = NULL;


    // Close I2C0
    //I2C0_Close();      
}