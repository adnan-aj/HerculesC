/*
 * packet.c
 *
 * Created: 2017-02-21 04:44:55 PM
 *  Author: Adnan
 */ 

#if 0

void doStatus(int error)
{
	
}

void setInt16toArrayBE(int v, unsigned char *b, unsigned char idx)
{
	b  += idx;
	*b++ = (v >> 8) & 0xff;
	*b   = (v >> 0) & 0xff;
}


void doSetSpeed(void)
{
	motorA_setting = getInt16FromArrayBE(payload, 2);
	motorB_setting = getInt16FromArrayBE(payload, 4);
	doStatus(STAT_OK);
}

void doGetSpeed(void)
{
	// Returns motor speed settings (not the inst. speed)
	payload[0] = robin_stat.lastcmd;
	payload[1] = 0;
	setInt16toArrayBE(motorA_setting, payload,  2);
	setInt16toArrayBE(motorB_setting, payload,  4);
	//robin_sendpkt(0, 0, 6, payload);
}

void doSetPID(void)
{
	int scaled_P, scaled_I, scaled_D;
	scaled_P = getInt16FromArrayBE(payload,  2);
	scaled_I = getInt16FromArrayBE(payload,  4);
	scaled_D = getInt16FromArrayBE(payload,  6);
	pid_Init(scaled_AP, scaled_AI, scaled_AD, &motorA_PidData);
	scaled_P = getInt16FromArrayBE(payload,  8);
	scaled_I = getInt16FromArrayBE(payload, 10);
	scaled_D = getInt16FromArrayBE(payload, 12);
	pid_Init(scaled_BP, scaled_BI, scaled_BD, &motorB_PidData);
	doStatus(STAT_OK);
}

void doGetPID(void)
{
	payload[0] = robin_stat.lastcmd;
	payload[1] = 0;
	setInt16toArrayBE(motorA_PidData.P_Factor, payload,  2);
	setInt16toArrayBE(motorA_PidData.I_Factor, payload,  4);
	setInt16toArrayBE(motorA_PidData.D_Factor, payload,  6);
	setInt16toArrayBE(motorB_PidData.P_Factor, payload,  8);
	setInt16toArrayBE(motorB_PidData.I_Factor, payload, 10);
	setInt16toArrayBE(motorB_PidData.D_Factor, payload, 12);
	robin_sendpkt(0, 0, 14, payload);
}

void doResetPID(void)
{
	// Uses factory-set PID constants
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &motorA_PidData);
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &motorB_PidData);
}

void doSavePID(void)
{
	// Saves the current PID into EEPROM
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *) 2, (motorA_PidData.P_Factor));
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *) 4, (motorA_PidData.I_Factor));
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *) 6, (motorA_PidData.D_Factor));
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *) 8, (motorB_PidData.P_Factor));
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *)10, (motorB_PidData.I_Factor));
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *)12, (motorB_PidData.D_Factor));
	doStatus(STAT_OK);
}

void doRecallPID(struct PID_DATA *motorA_PidData, struct PID_DATA *motorB_PidData)
{
	int scaled_P, scaled_I, scaled_D;
	eeprom_busy_wait();
	scaled_P = eeprom_read_word((uint16_t *)2);
	scaled_I = eeprom_read_word((uint16_t *)4);
	scaled_D = eeprom_read_word((uint16_t *)6);
	pid_Init(scaled_P, scaled_I, scaled_D, motorA_PidData);
	scaled_P = eeprom_read_word((uint16_t *)8);
	scaled_I = eeprom_read_word((uint16_t *)10);
	scaled_D = eeprom_read_word((uint16_t *)12);
	pid_Init(scaled_P, scaled_I, scaled_D, motorB_PidData);
}

void doPowerDown(void)
{
	// Go into power-saving mode for peripheral circuitries.
	// IF implemented, REMEMBER to activate power again.
}

void doResetOdo(void)
{
	motorA_odo = motorB_odo = 0;
	doStatus(STAT_OK);
}

void doPacketProcessing()
{
	int cmd = payload[0];

	if (cmd != 0)		// to check if it's a GetStatus command
	//robin_stat.lastcmd = cmd;
	switch(cmd) {
		case 0x00:	doStatus(STAT_OK);		break;
		case 0x01:	doGetStatusBySN();		break;
		case 0x02:	doSetNodeAddrBySN();	break;
		case 0x03:	doGetDeviceName();		break;
		case 0x04:	doGetDeviceVersion();	break;
		case 0x05:	doGetDeviceNotes();		break;
		case 0x06:	doSetDeviceNotes();		break;

		case 0x20: 	doAllStop();			break;
		case 0x21:	doGetStats();			break;
		case 0x22:	doSetSpeed();			break;
		case 0x23:	doGetSpeed();			break;
		case 0x24:	doSetPID();				break;
		case 0x25:	doGetPID();				break;
		case 0x26:	doSavePID();			break;
		case 0x27:	doResetPID();			break;
		case 0x28:	doResetOdo();			break;
		case 0x29:	doPowerDown();			break;
		
		default:	doStatus(STAT_ERROR);	break;
	}
}

#endif