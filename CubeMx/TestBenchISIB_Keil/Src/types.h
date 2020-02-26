
union Measures_telemetries
{
	struct{
		uint16_t ULoadCell;
		uint16_t Umotor;
		uint16_t Imotor;
		uint16_t Ubrake;
		uint16_t Ibrake;
		uint16_t Speed;
	}strct;
	uint8_t measures[10];
};

typedef union
{
	struct 
	{
		uint8_t SOF;
		uint8_t	OD_INDEX;
		uint8_t	Length;
		uint8_t*	data;
		uint8_t EOF;
	}fr;
	uint8_t frame[16]; 	
	
} T_SPI_FRAME;
