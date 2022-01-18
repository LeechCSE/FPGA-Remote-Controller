#include "platform.h"
#include "xparameters.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "xgpio.h"
#include "xil_io.h"

#include "xspi.h"
#include "xspi_l.h"
#include "PmodGYRO.h"

#include "xuartlite_l.h"
#include "xuartlite.h"


#define SWITCH_CHANNEL 1
#define SWITCH_ON 0x80
#define SWITCH_RUNNING 0xC0
#define SWITCH_INIT_ROBOT 0x82

#define LED_CHANNEL 1
#define LED_ON 0x80
#define LED_RUNNING 0xC0
#define LED_INITIALIZING 0xC1
#define LED_INIT_ROBOT 0x82

#define GYRO_BIAS_PERIOD 256 /* amount of readings used to compute bias */
#define GYRO_AVG_PERIOD 8 /* amount of readings used to compute most recent average */
#define GYRO_THRESHOLD 512  /* dps */
#define K_ROLL -256 //-1152 // -128
#define K_PITCH 512
#define K_YAW 128

#define OPCODE_DRIVE 137
#define MIN_VELOCITY -450
#define MAX_VELOCITY 450
#define MIN_RADIUS -1800
#define MAX_RADIUS 1800

#define MIN_V_THRESH 50
#define MIN_R_THRESH 200

#define THRESH_OUT_V 25
#define THRESH_OUT_R 50

/* x: roll
 * y: pitch
 * z: yaw
 */

#define blDvmaCR		0x00000000 // Control Reg Offset
#define blDvmaFWR		0x00000004 // Frame Width Reg Offset
#define blDvmaFHR		0x00000008 // Frame Height Reg Offset
#define blDvmaFBAR	0x0000000c // Frame Base Addr Reg Offset
#define blDvmaFLSR	0x00000010 // Frame Line Stride Reg Offeset
#define blDvmaHSR		0x00000014 // H Sync Reg Offset
#define blDvmaHBPR	0x00000018 // H Back Porch Reg Offset
#define blDvmaHFPR	0x0000001c // H Front Porch Reg Offset
#define blDvmaHTR		0x00000020 // H Total Reg Offset
#define blDvmaVSR		0x00000024 // V Sync Reg Offset
#define blDvmaVBPR	0x00000028 // V Back Porch Reg Offset
#define blDvmaVFPR	0x0000002c // V Front Porch Reg Offset
#define blDvmaVTR		0x00000030 // V Total Reg Offset

#define M_PI 3.14159265358979323846

#define HDMI_L_X 2560
#define HDMI_L_Y 2048

#define P_L_X 3
#define P_L_Y 1

#define MAP_L_X (HDMI_L_X >> P_L_X)
#define MAP_L_Y (HDMI_L_Y >> (P_L_Y))
#define MAP_K_P 128.0
#define MAP_K_R (32.0 / M_PI)

struct Position {
	float x;
	float y;
	float dir;
};


void InitDisplay() {
	int posX, posY;
	
	for(posX = 0; posX<HDMI_L_X; posX++){
		for(posY = 0; posY<HDMI_L_Y; posY++){
			XIo_Out16(XPAR_DDR2_SDRAM_MPMC_BASEADDR + 2 * (posY * HDMI_L_X + posX), 
				0xFFFF);
		}
	}

	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaHSR, 40); // hsync
	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaHBPR, 260); // hbpr
	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaHFPR, 1540); // hfpr
	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaHTR, 1650); // htr
	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaVSR, 5); // vsync
	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaVBPR, 25); // vbpr
	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaVFPR, 745); // vfpr
	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaVTR, 750); // vtr

	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaFWR, 0x00000500); // frame width
	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaFHR, 0x000002D0); // frame height
	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaFBAR, XPAR_DDR2_SDRAM_MPMC_BASEADDR); // frame base addr
	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaFLSR, 0x00000A00); // frame line stride
	XIo_Out32(XPAR_DVMA_0_BASEADDR + blDvmaCR, 0x00000003); // dvma enable, dfl enable
}

void MAP_UpdatePath(struct Position *pos, int16_t velocity, int16_t radius){
	int16_t r_straight = 0x8000;
	float n_x = pos->x;
	float n_y = pos->y;
	float n_dir = pos->dir;

	if (radius == r_straight){
		n_x += (1.0 * velocity / MAP_K_P) * cos(n_dir);
		n_y -= (1.0 * velocity / MAP_K_P) * sin(n_dir);
	}
	else if (radius == -1 || radius == 1)
		n_dir += radius * (1.0 * velocity / MAP_K_R);
	else {
		n_dir += (1.0 * velocity / (1.0 * radius * MAP_K_R));
		n_x += (1.0 * velocity / MAP_K_P) * cos(n_dir);
		n_y -= (1.0 * velocity / MAP_K_P) * sin(n_dir);
	}

	if (n_x < 0.0 || n_x >= 1.0 * MAP_L_X 
		|| n_y < 0.0 || n_y >= 1.0 * MAP_L_Y) // out-of-bound check
		return;

	float slope;
	float posX, posY;
	int mapX, mapY, outX, outY;
	if ((int)pos->x == (int)n_x){
		if (n_y >= pos->y){
			for (posX = n_x, posY = pos->y; posY < n_y; ++posY){
				mapX = (int)posX;
				mapY = (int)posY;
				
				for (outX = mapX << (P_L_X - 1); outX < ((mapX + 1) << (P_L_X - 1)); outX++){
					for (outY = mapY << P_L_Y; outY < ((mapY + 1) << P_L_Y); outY++){
						XIo_Out16(XPAR_DDR2_SDRAM_MPMC_BASEADDR + 2 * (outY * HDMI_L_X + outX), 
							0x0000);
					}
				}
			}
		}
		else{
			for (posX = n_x, posY = pos->y; posY > n_y; --posY){
				mapX = (int)posX;
				mapY = (int)posY;
				
				for (outX = mapX << (P_L_X - 1); outX < ((mapX + 1) << (P_L_X - 1)); outX++){
					for (outY = mapY << P_L_Y; outY < ((mapY + 1) << P_L_Y); outY++){
						XIo_Out16(XPAR_DDR2_SDRAM_MPMC_BASEADDR + 2 * (outY * HDMI_L_X + outX), 
							0x0000);
					}
				}
			}
		}
	}
	else if (pos->x < n_x){
		slope = (n_y - pos->y) / (n_x - pos->x);
		for (posX = pos->x, posY = pos->y; posX <= n_x; posX++, posY += slope){
			mapX = (int)posX;
			mapY = (int)posY;
			
			for (outX = mapX << (P_L_X - 1); outX < ((mapX + 1) << (P_L_X - 1)); outX++){
				for (outY = mapY << P_L_Y; outY < ((mapY + 1) << P_L_Y); outY++){
					XIo_Out16(XPAR_DDR2_SDRAM_MPMC_BASEADDR + 2 * (outY * HDMI_L_X + outX), 
						0x0000);
				}
			}
		}
	} 
	else {
		slope = (n_y - pos->y) / (n_x - pos->x);
		for (posX = pos->x, posY = pos->y; posX >= n_x; posX--, posY += slope){
			mapX = (int)posX;
			mapY = (int)posY;
			
			for (outX = mapX << (P_L_X - 1); outX < ((mapX + 1) << (P_L_X - 1)); outX++){
				for (outY = mapY << P_L_Y; outY < ((mapY + 1) << P_L_Y); outY++){
					XIo_Out16(XPAR_DDR2_SDRAM_MPMC_BASEADDR + 2 * (outY * HDMI_L_X + outX), 
						0x0000);
				}
			}
		}
	}

	pos->x = n_x;
	pos->y = n_y;
	pos->dir = n_dir;
}

void MAP_ClearPath(struct Position *pos){
	pos->dir = M_PI / 2.0;
	pos->x = MAP_L_X / 2.0;
	pos->y = MAP_L_Y / 6.0;
	
	for(int posX = 0; posX<HDMI_L_X; posX++){
		for(int posY = 0; posY<HDMI_L_Y; posY++){
			XIo_Out16(XPAR_DDR2_SDRAM_MPMC_BASEADDR + 2 * (posY * HDMI_L_X + posX), 
				0xFFFF);
		}
	}
}

void RBT_SendDriveCmd(int16_t velocity, int16_t radius){
	union s16to2u8 {
		int16_t s;
		uint8_t b[2];
	}v, r;
	
	v.s = velocity;
	r.s = radius;
	
	XUartLite_SendByte(XPAR_XPS_UARTLITE_0_BASEADDR, OPCODE_DRIVE);
	XUartLite_SendByte(XPAR_XPS_UARTLITE_0_BASEADDR, v.b[0]);
	XUartLite_SendByte(XPAR_XPS_UARTLITE_0_BASEADDR, v.b[1]);
	XUartLite_SendByte(XPAR_XPS_UARTLITE_0_BASEADDR, r.b[0]);
	XUartLite_SendByte(XPAR_XPS_UARTLITE_0_BASEADDR, r.b[1]);
}

struct ReadingNode{
	int16_t val;
	struct ReadingNode *next;
};

struct ReadingQueue{
	struct ReadingNode *head;
	struct ReadingNode nodes[GYRO_AVG_PERIOD];
};

void RQ_Init(struct ReadingQueue *rq){
	rq->head = &(rq->nodes[0]);
	
	for (uint16_t i = 0; i < GYRO_AVG_PERIOD - 1; i++){
		rq->nodes[i].val = 0;
		rq->nodes[i].next = &(rq->nodes[i + 1]);
	}
	
	rq->nodes[GYRO_AVG_PERIOD - 1].val = 0;
	rq->nodes[GYRO_AVG_PERIOD - 1].next = rq->head;
}

void RQ_Push(struct ReadingQueue *rq, int16_t val){
	rq->head->val = val;
	rq->head = rq->head->next;
}

int16_t RQ_Avg(struct ReadingQueue *rq){
	int32_t sum = 0;
	
	for (uint16_t i = 0; i < GYRO_AVG_PERIOD; i++)
		sum += rq->nodes[i].val;
	
	return (int16_t)(sum / GYRO_AVG_PERIOD);
}


int GYRO_InitSpi(XSpi *InstancePtr){
	int Status;

	Status = XSpi_Initialize(InstancePtr, XPAR_SPI_GYRO_DEVICE_ID);
	if (Status != XST_SUCCESS){
	   //xil_printf("SPI config failed.\r\n");
	  return XST_FAILURE;
	}

	Status = XSpi_SetOptions(InstancePtr, (XSP_MASTER_OPTION
		 | XSP_CLK_ACTIVE_LOW_OPTION | XSP_CLK_PHASE_1_OPTION
		 | XSP_MANUAL_SSELECT_OPTION));
	if (Status != XST_SUCCESS){
	   //xil_printf("SPI setoptions failed.\r\n");
	  return XST_FAILURE;
	}
	/*
	* Start the SPI driver so that the device is enabled.
	*/
	XSpi_Start(InstancePtr);

	/*
	* Disable Global interrupt to use polled mode operation
	*/
	XSpi_IntrGlobalDisable(InstancePtr);

	Status = XSpi_SetSlaveSelect(InstancePtr, 1);
	if (Status != XST_SUCCESS){
	   //xil_printf("SPI set slave select failed.\r\n");
	  return XST_FAILURE;
	}

	return XST_SUCCESS;
}

u8 GYRO_ReadRegByte(XSpi *InstancePtr, u8 reg){
	// As requested by documentation, first byte contains:
	//    bit 7 = 1 because is a read operation
	//    bit 6 = 1 if more than one byte is written, 0 if one byte is written
	//    bits 5-0 - the address

	u8 buf_send[2] = {0x80 | (reg & 0x3F), 0};
	u8 buf_recv[2] = {0};
	int rv = XSpi_Transfer(InstancePtr, buf_send, buf_recv, 2);
	//if (rv == XST_FAILURE) xil_printf("ERROR: spi transfer when reading from reg failed\r\n");
	//xil_printf("reg out: buf[0] = 0x%02x, buf[1] = 0x%02x\r\n", buf_recv[0], buf_recv[1]);
	return buf_recv[1];
}

void GYRO_WriteRegByte(XSpi *InstancePtr, u8 reg, u8 w_data){
	// As requested by documentation, first byte contains:
	//    bit 7 = 0 because is a write operation
	//    bit 6 = 1 if more than one byte is written, 0 if one byte is written
	//    bits 5-0 - the address

	u8 buf_send[2] = {reg & 0x3F, w_data};
	int rv = XSpi_Transfer(InstancePtr, buf_send, NULL, 2); /* only sending */
	//if (rv == XST_FAILURE) xil_printf("ERROR: spi transfer when writing to reg failed\r\n");
	/*
	u8 test_byte = GYRO_ReadRegByte(InstancePtr, reg);
	xil_printf("wrote 0x%02x to register\r\n", w_data);
	xil_printf("data in register after write: 0x%02x\r\n", test_byte);
	*/
}

void GYRO_InitCtrlRegs(XSpi *InstancePtr){
	u8 w_byte = 0x80;
	GYRO_WriteRegByte(InstancePtr, GYRO_CTRL_REG5, w_byte);
	w_byte = (GYRO_REG1_PD | GYRO_REG1_XEN | GYRO_REG1_YEN | GYRO_REG1_ZEN);
	GYRO_WriteRegByte(InstancePtr, GYRO_CTRL_REG1, w_byte);
	w_byte = 0x40;
	GYRO_WriteRegByte(InstancePtr, GYRO_CTRL_REG5, w_byte);
	w_byte = GYRO_STREAM; /* stream mode with no watermark set */
	GYRO_WriteRegByte(InstancePtr, GYRO_FIFO_CTRL_REG, w_byte);
}

int16_t GYRO_GetRoll(XSpi *InstancePtr){
	uint16_t temp_data = GYRO_ReadRegByte(InstancePtr, GYRO_OUT_X_H);
	return (int16_t)((temp_data << 8) | GYRO_ReadRegByte(InstancePtr, GYRO_OUT_X_L));
}

int16_t GYRO_GetPitch(XSpi *InstancePtr){
	uint16_t temp_data = GYRO_ReadRegByte(InstancePtr, GYRO_OUT_Y_H);
	return (int16_t)((temp_data << 8) | GYRO_ReadRegByte(InstancePtr, GYRO_OUT_Y_L));
}

int16_t GYRO_GetYaw(XSpi *InstancePtr){
	uint16_t temp_data = GYRO_ReadRegByte(InstancePtr, GYRO_OUT_Z_H);
	return (int16_t)((temp_data << 8) | GYRO_ReadRegByte(InstancePtr, GYRO_OUT_Z_L));
}

u8 GYRO_CheckData(XSpi *InstancePtr){
	return GYRO_ReadRegByte(InstancePtr, GYRO_STATUS_REG) & 0x08;
}

void GYRO_GetBias(XSpi *InstancePtr, int16_t *bias_roll, int16_t *bias_pitch, int16_t *bias_yaw) {
	long b_r = 0, b_p = 0, b_y = 0;
	
	for(int i = 0; i < GYRO_BIAS_PERIOD;){
		if (GYRO_CheckData(InstancePtr)){
			b_r += GYRO_GetRoll(InstancePtr);
			b_p += GYRO_GetPitch(InstancePtr);
			b_y += GYRO_GetYaw(InstancePtr);
			i++;
		}
	}
	
	*bias_roll = (int16_t)(b_r / GYRO_BIAS_PERIOD);
	*bias_pitch = (int16_t)(b_p / GYRO_BIAS_PERIOD);
	*bias_yaw = (int16_t)(b_y / GYRO_BIAS_PERIOD);
}


int main(){
	init_platform();
	//xil_printf("starting...\r\n");

	InitDisplay();

	XGpio sw_gpio;
	if (XGpio_Initialize(&sw_gpio, XPAR_FPGA_0_DIP_SWITCHES_8BIT_DEVICE_ID) != XST_SUCCESS){
		//xil_printf("failed to init gpio switches\r\n");
		return XST_FAILURE;
	}
	XGpio_SetDataDirection(&sw_gpio, SWITCH_CHANNEL, 0xFF);

	XGpio led_gpio;
	if (XGpio_Initialize(&led_gpio, XPAR_FPGA_0_LEDS_8BIT_DEVICE_ID) != XST_SUCCESS){
		//xil_printf("failed to init gpio leds\r\n");
		return XST_FAILURE;
	}
	XGpio_SetDataDirection(&led_gpio, LED_CHANNEL, 0x00);

	XSpi gyro_spi;

	if (GYRO_InitSpi(&gyro_spi) == XST_FAILURE){
		//xil_printf("failed to init SPI peripheral for gyroscope\r\n");
		return XST_FAILURE;
	}
	GYRO_InitCtrlRegs(&gyro_spi);

	int16_t bias_roll, bias_pitch, bias_yaw;
	struct ReadingQueue roll_q;
	struct ReadingQueue pitch_q;
	struct ReadingQueue yaw_q;
	int16_t avg_roll, avg_pitch;//, avg_yaw;
	int16_t radius, velocity, prev_out_v = 0, prev_out_r = 0;

	struct Position pos;

	/* switches are ACTIVE LOW due to the board being turned around when in use */
	while(XGpio_DiscreteRead(&sw_gpio, SWITCH_CHANNEL) & SWITCH_ON){} /*spin until ON switch flipped */

	while(!(XGpio_DiscreteRead(&sw_gpio, SWITCH_CHANNEL) & SWITCH_ON)){ /* looping main routine while ON switch is flipped */
		XGpio_DiscreteWrite(&led_gpio, LED_CHANNEL, LED_ON);

		if (!(XGpio_DiscreteRead(&sw_gpio, SWITCH_CHANNEL) & SWITCH_INIT_ROBOT)){
			XGpio_DiscreteWrite(&led_gpio, LED_CHANNEL, LED_INIT_ROBOT);
			/* initialize irobot */
			XUartLite_SendByte( XPAR_XPS_UARTLITE_0_BASEADDR, 0x80 ); // init
			XUartLite_SendByte( XPAR_XPS_UARTLITE_0_BASEADDR, 0x84 ); // full mode
			for (int j = 0; j < 2; j++){
				for (int i = 0; i < 25000000; i++){
					asm("nop");
				}
			}
			
			XGpio_DiscreteWrite(&led_gpio, LED_CHANNEL, LED_ON);
		}

		if (!(XGpio_DiscreteRead(&sw_gpio, SWITCH_CHANNEL) & SWITCH_RUNNING)){ /* if RUNNING switches are flipped, intialize */
			XGpio_DiscreteWrite(&led_gpio, LED_CHANNEL, LED_INITIALIZING); /* flag leds, tells user to hold the FPGA level */

			GYRO_GetBias(&gyro_spi, &bias_roll, &bias_pitch, &bias_yaw);
			RQ_Init(&roll_q);
			RQ_Init(&pitch_q);
			radius = 0;
			velocity = 0;

			MAP_ClearPath(&pos);

			uint8_t cnt = 0;
			XGpio_DiscreteWrite(&led_gpio, LED_CHANNEL, LED_RUNNING); /* flag leds */
			while (!(XGpio_DiscreteRead(&sw_gpio, SWITCH_CHANNEL) & SWITCH_RUNNING)){ /* reading & transmitting while RUNNING switches are flipped */
				if (GYRO_CheckData(&gyro_spi)){
					RQ_Push(&roll_q, GYRO_GetRoll(&gyro_spi));
					RQ_Push(&pitch_q, GYRO_GetPitch(&gyro_spi));
					RQ_Push(&yaw_q, GYRO_GetYaw(&gyro_spi));

					avg_roll = RQ_Avg(&roll_q);
					avg_pitch = RQ_Avg(&pitch_q);
					//avg_yaw = RQ_Avg(&yaw_q);

					if (abs(avg_roll) > GYRO_THRESHOLD)
						radius += avg_roll / K_ROLL;
					/*
					if (abs(avg_yaw) > GYRO_THRESHOLD) {
						theta += avg_yaw / K_YAW;
					}
					*/
					if (abs(avg_pitch) > GYRO_THRESHOLD)
						velocity += avg_pitch / K_PITCH;

					if (++cnt == 8) {
						cnt = 0;
						int16_t out_radius, out_velocity;
						
						if (velocity < MIN_VELOCITY) 				out_velocity = MIN_VELOCITY;
						else if (velocity > MAX_VELOCITY) 		out_velocity = MAX_VELOCITY;
						else if (abs(velocity) < MIN_V_THRESH) out_velocity = 0;
						else 												out_velocity = velocity;

						if (radius < MIN_RADIUS) 					out_radius = -1;
						else if (radius > MAX_RADIUS) 			out_radius = 1;
						else if (abs(radius) < MIN_R_THRESH) 	out_radius = 0x8000;
						else if (radius < 0) 						out_radius = -1800 - radius;
						else 												out_radius = 1800 - radius;
						
						if (abs(prev_out_v - out_velocity) > THRESH_OUT_V 
							|| abs(prev_out_r - out_radius) > THRESH_OUT_R){
							RBT_SendDriveCmd(out_velocity, out_radius);
							prev_out_v = out_velocity;
							prev_out_r = out_radius;
						}
						
						MAP_UpdatePath(&pos, prev_out_v, prev_out_r);
					}
				}
			} /* RUNNING switch flipped off, will loop back up */

			RBT_SendDriveCmd(0,0);
		}
	}

	RBT_SendDriveCmd(0,0);
	XGpio_DiscreteWrite(&led_gpio, LED_CHANNEL, 0x00);
	XSpi_Stop(&gyro_spi);
	
	return 0;
}
