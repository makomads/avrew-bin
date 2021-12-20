#define	F_CPU  18000000UL 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define	sbi(PORT,BIT) PORT |= _BV(BIT)	
#define	cbi(PORT,BIT) PORT &=~_BV(BIT)
#define false 0
#define true 1



#define SOFTUART_BAUDRATE 9600L
#define SOFTUART_TX_PORT PORTB
#define SOFTUART_TX_PINNO 6
#define SOFTUART_RX_PIN PINB
#define SOFTUART_RX_PINNO 5
#define SOFTUART_WAIT (F_CPU/SOFTUART_BAUDRATE/6-1)




void UARTSendByte(uint8_t data)
{
	uint8_t bitcnt;
	uint16_t framedata;

	//[debug]
	//data = 0x53;

	cli();

	//スタート、ストップビットを付加
	framedata = data;
	framedata <<= 1;
	framedata |= 0x0200;
	framedata &= 0xFFFE;
	//LSBから順に送出
	bitcnt = 10;
	do{
		if(framedata&1)
			sbi(SOFTUART_TX_PORT, SOFTUART_TX_PINNO);
		else
			cbi(SOFTUART_TX_PORT, SOFTUART_TX_PINNO);
		framedata >>= 1;
		_delay_us(34);
		_delay_us(34);
		_delay_us(35);
		bitcnt--;
	}while(bitcnt);

	sei();
}



volatile uint8_t sua_rxdata = 0;
volatile uint8_t sua_frxcomp = 0;	//ソフトUART受信完了フラグ

ISR(PCINT_vect)
{
	uint8_t bitcnt;

	//UARTはアイドル時High、スタートビットは立下りを検出
	if( bit_is_set(SOFTUART_RX_PIN, SOFTUART_RX_PINNO) )
		return;

	cli();

	//スタートビットのパルス中央になるまで待つ
	_delay_us(26);
	_delay_us(26);
	sua_rxdata = 0;
	//LSBから順に採取
	bitcnt = 8;
	do{
		_delay_us(34);
		_delay_us(34);
		_delay_us(35);
		sua_rxdata >>= 1;
		if( bit_is_set(SOFTUART_RX_PIN, SOFTUART_RX_PINNO) )
			sua_rxdata |= 0x80;
		bitcnt--;
	}while(bitcnt);

	//ストップビットを読み飛ばす
	_delay_us(34);
	_delay_us(34);
	_delay_us(35);


	sua_frxcomp = true;

	//ピン変化割り込みフラグ解除
	EIFR = 1<<PCIF;
	//マスター割り込み許可
	sei();

}



int main(void) 
{
/*
       ┏━━━┓
~RESET ┃A2  Vc┃Vcc
       ┃D0  B7┃(USCK)
       ┃D1  B6┃SoftTx(MISO)
   XTL ┃A1  B5┃SoftRx(MOSI)
   XTL ┃A0  B4┃
       ┃D2  B3┃out2
       ┃D3  B2┃in2
       ┃D4  B1┃out1
       ┃D5  B0┃in1
   GND ┃GN  D6┃
       ┗━━━┛

*/
	uint8_t uartbuf[20];
	uint8_t bufsize=20;
	uint8_t bufidx=0;
	uint8_t i;

	//マスター割り込み禁止
	cli();

	//IO方向セット(0入力,1出力)
	DDRB = 0b01001010;
	DDRD = 0b00000000;

	//ポート初期化
	//出力の場合1でHigh、0でLow
	//入力の場合1を書くとプルアップ抵抗活性化、0を書くとハイインピーダンス
	PORTB = 0b01100000;
	PORTD = 0b00000000;
	
	
	//////////////////////////////////////////////////////////////////////
	//ピン割り込み
	
	//ピン変化割り込みマスク設定
	PCMSK = 1<<PCINT5;
	//ピン割り込み許可
	GIMSK = 1<<PCIE;


	//////////////////////////////////////////////////////////////////////
	
	//諸変数初期化
	sua_frxcomp = false;

	//[debug]
	//UARTSendByte(0x55);

	//割り込み許可
	sei();

	//メインループ
	while(1){
		if(bit_is_set(PINB,0))
			sbi(PORTB,1);
		else
			cbi(PORTB,1);
		if(bit_is_set(PINB,2))
			sbi(PORTB,3);
		else
			cbi(PORTB,3);
		
		//ソフトUART受信完了待ち
		if(sua_frxcomp){
			sua_frxcomp = false;
			uartbuf[bufidx++] = sua_rxdata;
		}
		//返信
		if(sua_rxdata=='\n' || bufidx==bufsize){
			for(i=0; i<bufidx; i++){
				if(uartbuf[i]>=0x20)
					UARTSendByte(uartbuf[i]+1);
			}
			bufidx=0;
		}
			

	}

	return 0;
}

