/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
enum { CLASS_MON = 0x0A };
/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
enum { CLASS_MON_RF = 0x38 , CLASS_MON_SPAN = 0x31 };
/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
typedef     UBYTE   JAMMING_STATE <read=jamming_state>;
typedef     UBYTE   ANT_STATUS    <read=ant_status>;
typedef     UBYTE   ANT_POWER     <read=ant_power>;

string jamming_state( JAMMING_STATE v )
{
    switch ( v ) {
    case 0 : return "Unknow or feature disabled";
    case 1 : return "OK - no significant jamming";
    case 2 : return "Warning interference visible but fix OK";
    case 3 : return "Critical-interference visible and no fix";
    }
    return "Reserved";
}

string ant_status( ANT_STATUS v )
{
    switch ( v ) {
    case 0 : return "INIT";
    case 1 : return "DONT'KNOW";
    case 2 : return "OK";
    case 3 : return "SHORT";
    case 4 : return "OPEN";
    }
    return "Reserved";
}

string ant_power( ANT_POWER v )
{
    switch ( v ) {
    case 0 : return "OFF";
    case 1 : return "ON";
    case 2 : return "DONT'KNOW";
    }
    return "Reserved";
}

/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
typedef struct {
    UBYTE       blockId;
    // UBYTE       flags;          // bit 0-1 : jammingState
    struct {
        JAMMING_STATE jammingState : 2;
        UBYTE   _reserved : 6;
    } flags;
    ANT_STATUS  antStatus;      // 2 = OK , 3=Short , 4=Open
    ANT_POWER   antPower;       // 1 = On ( Active Antenna)
    UINT32      postStatus;
    UINT32      _reserved1;
    UINT16      noisePerMS;
    UINT16      agcCnt;         // Gain
    UINT8       jamInd;         // 0-255 noise index
    INT8        ofsI;
    UINT8       magI;
    INT8        ofsQ;
    UINT8       magQ;
    UINT8       _reserved2[3];
} UBX_MON_RF_BLOCK_T;

typedef struct {
    // UBYTE           raw[ 28 ];
    UINT8       version;
    UINT8       nBlocks;
    UINT16      reserved;
    UBX_MON_RF_BLOCK_T rf_blk;      // RF block
} UBX_MON_RF;

/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
typedef UBYTE   SPECTRUM    <read=spectrum_display>;

string spectrum_display ( SPECTRUM v )
{
    float x = v;

    x = ( x / 255.0 ) * 60.0;
    return Str("%.1f dB.",x );
}

typedef struct {
    // UINT8       spectrum[256];
    SPECTRUM    spectrum[256];
    UINT32      span;
    UINT32      res;
    UINT32      center;
    UINT8       pga;
    UINT8       reserved1[3];
} UBX_MON_SPAN_RF_BLOCKS;

typedef struct {
    UINT8       version;
    UINT8       numRfBlocks;
    UINT8       reserved0[2];
    UBX_MON_SPAN_RF_BLOCKS  rfblock[numRfBlocks];
} UBX_MON_SPAN;


