// Useful debugging info
void dbg_uartout(uint8_t * inBuf, uint8_t * outBuf, uint8_t * msg)
{
	if(outBuf[0] == 62)	// SSMM
	{
		msg = "SMM:\t";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	}
	else if(outBuf[0] == 78)	// RM
	{
		msg = "RM:\t";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	}
	else if(outBuf[0] == 240)	// RST
	{
		msg = "RST:\t";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	}
	else if(outBuf[0] == 80)	// RR
	{
		msg = "RR:\t";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	}
	if(inBuf[0] & 128)	msg = "BM\t";
	else				msg = "  \t";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	if(inBuf[0] & 64)	msg = "WC\t";
	else				msg = "  \t";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	if(inBuf[0] & 32)	msg = "SM\t";
	else				msg = "  \t";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	if(inBuf[0] & 16)	msg = "ER\t";
	else				msg = "  \t";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	if(inBuf[0] & 8)	msg = "SD\t";
	else				msg = "  \t";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	if(inBuf[0] & 4)	msg = "RS\t";
	else				msg = "  \t";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	if(inBuf[0] & 2)	msg = "D1\t";
	else				msg = "  \t";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	if(inBuf[0] & 1)	msg = "D0\t";
	else				msg = "  \t";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	if(HAL_GPIO_ReadPin(COMP_INT_P, COMP_INT))
	{
		msg = "RDY\t";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	}
	msg = "\n\n\r";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
}

// I/O
void send_SPI(uint8_t * inBuf, uint8_t inLength, uint8_t * outBuf, uint8_t outLength)
{
	HAL_GPIO_WritePin(COMP_CS_P, COMP_CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, outBuf, outLength, 1000);
	HAL_SPI_Receive(&hspi2, inBuf, inLength, 1000);
	HAL_GPIO_WritePin(COMP_CS_P, COMP_CS, GPIO_PIN_SET);
}

// Print results as integers
void compass_uart(int16_t * xyz, uint8_t * msg, int16_t * angle)
{
	sprintf(msg, "x: %5d\t", xyz[0]);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	sprintf(msg, "y: %5d\t", xyz[1]);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	sprintf(msg, "z: %5d\t", xyz[2]);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	sprintf(msg, "a: %d\t", *angle);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	msg = "\n\r";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
}

// Convert raw input bits to integers
void processInput(uint8_t * inbuf, int16_t * xyz, int16_t * minmaxxyz)
{
	// FL is off by 2 degrees W: Don't care.
	for(uint8_t i=0; i<3; i++)
	{
		// 2's complement binaries to int16_t
		xyz[i] = ((int16_t)inbuf[i*2+1] << 8 | inbuf[i*2+2]);
		// (optional) use GAIN_SEL to find "real" value
		xyz[i] *= i==2?.294:.161;
		// Update minmaxxyz
		if(xyz[i] < minmaxxyz[2*i])	minmaxxyz[2*i] = xyz[i];
		if(xyz[i] > minmaxxyz[2*i+1])	minmaxxyz[2*i+1] = xyz[i];
		// Normalize results
		xyz[i] -= (minmaxxyz[2*i+1]+minmaxxyz[2*i])/2;
	}
}

/*
 * Calibration: Do a bunch of measurements
 *
 * Measurements/normalization: Save min and max. Med = 0
 */

void calibrate(uint8_t * inbuf, uint8_t * outbuf, int16_t * xyz, int16_t * minmaxxyz)
{
	takemeasurement(inbuf, outbuf);
	for(uint8_t i=0; i<3; i++)
	{
		// 2's complement binaries to int16_t
		xyz[i] = ((int16_t)inbuf[i*2+1] << 8 | inbuf[i*2+2]);
		// (optional) use GAIN_SEL to find "real" value
		xyz[i] *= i==2?.294:.161;
		// Update minmaxxyz
		minmaxxyz[2*i] = xyz[i];
		minmaxxyz[2*1+1] = xyz[i];
	}
	for(uint16_t i=0; i<100; i++)	// TODO replace this with pushbutton
	{
		takemeasurement(inbuf, outbuf);
		processInput(inbuf, xyz, minmaxxyz);
		osDelay(100);
	}
}

void takemeasurement(uint8_t * inbuf, uint8_t * outbuf)
{
	  // Start single measurement mode
	  outbuf[0] = 62;	// 0011zyxt
	  send_SPI(inbuf, 1, outbuf, 1);
	  //dbg_uartout(inbuf, outbuf, msg);

	  while(!HAL_GPIO_ReadPin(COMP_INT_P, COMP_INT));

	  //Read measurement
	  outbuf[0] = 78;
	  send_SPI(inbuf, 7, outbuf, 1);
	  //dbg_uartout(inbuf, outbuf, msg);
}

void getAngle(int16_t * angle, int16_t * xyz)
{
	volatile double tmp = atan((double)xyz[0]/(double)xyz[1]);
	tmp *= (180/3.1428);
	tmp = 90 - tmp;
	if(xyz[1] > 0)	*angle = (int16_t)(90.0 - (atan((double)xyz[0]/(double)xyz[1]) * (180.0 / 3.1428)));
	else if(xyz[1] < 0)	*angle = (int16_t)(270.0 - (atan((double)xyz[0]/(double)xyz[1]) * (180.0 / 3.1428)));
	else if(xyz[0]<0)	*angle = 180;
	else	*angle = 0;
}