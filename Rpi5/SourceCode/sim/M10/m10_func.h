/*------------------------------------------------------------------------

 ------------------------------------------------------------------------*/
typedef INT32   POSIT_SCALE     <read=PositionScale>;
typedef INT32   ANGLE_SCALE     <read=AngleScale>;
typedef INT16   PDOP_SCALE      <read=pdopScale>;
typedef INT32   MET_SCALE       <read=meterScale>;
typedef INT32   MET_SEC_SCALE   <read=meterSecScale>;

typedef INT16   ANGLE_I16_SCALE <read=AngleI16Scale>;
typedef UINT16  ANGLE_U16_SCALE <read=AngleU16Scale>;

string PositionScale( POSIT_SCALE v )
{
    string s;
    float f = v;
    f = f / 10000000.0;  // scale

    SPrintf( s , "%f dec." ,  f );
    return s;
}

string AngleScale( ANGLE_SCALE v )
{
    string s;
    float f = v;
    f = f / 100000.0;  // scale

    SPrintf( s , "%f deg." ,  f );
    return s;
}

string AngleI16Scale( ANGLE_I16_SCALE v )
{
    string s;
    float f = v;
    f = f / 100000.0;  // scale

    SPrintf( s , "%f deg." ,  f );
    return s;
}

string AngleU16Scale( ANGLE_U16_SCALE v )
{
    string s;
    float f = v;
    f = f / 100000.0;  // scale

    SPrintf( s , "%f deg." ,  f );
    return s;
}

string meterScale( MET_SCALE v )
{
    string s;
    float f = v;
    f = f / 1000.0;  // scale

    SPrintf( s , "%f m." ,  f );
    return s;
}

string meterSecScale( MET_SEC_SCALE v )
{
    string s;
    float f = v;
    f = f / 1000.0;  // scale

    SPrintf( s , "%f m/s." ,  f );
    return s;
}

string pdopScale( PDOP_SCALE v )
{
    string s;
    float f = v;
    f = f / 100.0;  // scale

    SPrintf( s , "%f." ,  f );
    return s;
}

