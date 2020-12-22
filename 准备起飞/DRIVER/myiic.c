#include "myiic.h"
#include "delay.h"

//初始化IIC
void IIC_Init(void)
{			
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
  GPIO_SetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9);//
	IIC_SCL=1;
	IIC_SDA=1;
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
//在CPU向外设写完一个字节时,等待外设的应答信号,该信号由外设自动发送,是在SCL为高时,SDA拉低代表已收到.
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入,CPU发完1个字节数据后就暂时不需要再驱动总线了,SDA变输入;输入是相对于CPU来说的,即外设向CPU输入信息
	IIC_SDA=1;delay_us(1);	  //因为现在是SDA输入到CPU,所以即使SDA从高变为低,也不能算是新的开始信号!  开始\结束信号只能由CPU发出(SDA_OUT时才可) 
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)  //READ_SDA是PBin(9),当READ_SDA为0时,表示CPU已经接收到应答信号,退出等待
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答,接收端(外设)拉低
//CPU从外设读数据时,外设传完1字节数据后,CPU要做出应答信号
//当SCL为高电平时,SDA为低电平并保持一段时间,代表CPU已经成功收到来自外设的数据
void IIC_Ack(void)
{
	IIC_SCL=0;   //先将SCL拉低,防止变成了开始信号
	SDA_OUT();  //CPU应答.SDA变输出
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;  //将SCL拉高,CPU开始发送ACK应答信号
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;  //SDA始终为高电平,代表CPU不应答
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
//CPU向外设发送数据
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
        IIC_SDA=(txd&0x80)>>7;  //从高位开始传送
				txd<<=1; 	       //不断将低位左移到高位
				delay_us(2);   //对TEA5767这三个延时都是必须的(TEA5767是收音机芯片)
				IIC_SCL=1;     //开始传数据
				delay_us(2); 
				IIC_SCL=0;	//为下bit数据准备
				delay_us(2);
    }
} 	    


//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++)
		{
        IIC_SCL=0; 
        delay_us(2);    //等待外设设置SDA
				IIC_SCL=1;
        receive<<=1;   //外设从高位开始向CPU传送数据
				if(READ_SDA)receive++;   //READ_SDA等价于PBin(9),即SDA!当SDA为高电平(1)时,receive的最低为置1
				delay_us(1); 
    }					 
    if (!ack)   //ack只是CPU是否接收外设数据的标志,而存储外设传来的数据的是receive
			IIC_NAck();//发送nACK, 不应答
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

//读两个字节
u8 IIC_Read_2Bytes(unsigned char ack)
{
	unsigned char i;
	u16 receive = 0;
	SDA_IN();//SDA设置为输入
	for(i = 0; i < 16; i++)
	{
		IIC_SCL = 0;
		delay_us(2);  //等待外设设置SDA
		IIC_SCL = 1;
		receive <<= 1; //从高位开始传输数据，第一次是初始化
		if(READ_SDA)
			receive++;
		delay_us(1);
	}
	
	if (!ack)   //ack只是CPU是否接收外设数据的标志,而存储外设传来的数据的是receive
    IIC_NAck();//发送nACK
  else
    IIC_Ack(); //发送ACK   
  return receive;
}

//读三个字节
u8 IIC_Read_3Bytes(unsigned char ack)
{
	unsigned char i;
	u32 receive = 0;
	SDA_IN();//SDA设置为输入
	for(i = 0; i < 24; i++)
	{
		IIC_SCL = 0;
		delay_us(2);  //等待外设设置SDA
		IIC_SCL = 1;
		receive <<= 1; //从高位开始传输数据，第一次是初始化
		if(READ_SDA)
			receive++;
		delay_us(1);
	}
	
	if (!ack)   //ack只是CPU是否接收外设数据的标志,而存储外设传来的数据的是receive
    IIC_NAck();//发送nACK
  else
    IIC_Ack(); //发送ACK   
  return receive;
}

