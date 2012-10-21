
#define PXA_310_LV

#include <linux/kernel.h>
#include <asm/irq.h>
#include <mach/hardware.h>

#include <linux/init.h>
#include <linux/module.h>
#ifdef PXA_310_LV
#include <mach/ssp.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/gpio.h>
#include <mach/pxa910_pm.h>
#include <mach/regs-ssp.h>
///
#endif
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <mach/dma.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <asm/io.h>
#include "smsdbg_prn.h"

#ifdef PXA_310_LV


#define SSP_PORT 1
#define SSP_CKEN CKEN_SSP1

#define  SSP_FIFO   (0xD401B000+0x10)

#define mfp_to_gpio(m)	((m) % 256)
#define APB_VIRT_BASE		0xfe000000
#define GPIO_REGS_VIRT	(APB_VIRT_BASE + 0x19000)

#define BANK_OFF(n)	(((n) < 3) ? (n) << 2 : 0x100 + (((n) - 3) << 2))
#define GPIO_REG(x)	(*((volatile u32 *)(GPIO_REGS_VIRT + (x))))

#define NR_BUILTIN_GPIO	(128)

#define gpio_to_bank(gpio)	((gpio) >> 5)
#define gpio_to_irq(gpio)	(IRQ_GPIO_START + (gpio))
#define irq_to_gpio(irq)	((irq) - IRQ_GPIO_START)
#define GCRER(x)        GPIO_REG(BANK_OFF(gpio_to_bank(x)) + 0x78)
#define GSRER(x)        GPIO_REG(BANK_OFF(gpio_to_bank(x)) + 0x6c)

#define GPSR(x)		GPIO_REG(BANK_OFF(gpio_to_bank(x)) + 0x18)
#define GPCR(x)		GPIO_REG(BANK_OFF(gpio_to_bank(x)) + 0x24)
#define GPDR(x)		GPIO_REG(BANK_OFF(gpio_to_bank(x)) + 0x0c)

#define GSDR(x)		GPIO_REG(BANK_OFF(gpio_to_bank(x)) + 0x54)
#define GCDR(x)		GPIO_REG(BANK_OFF(gpio_to_bank(x)) + 0x60)
//--------------------------------------------------------------------------------------------

#if (SSP_PORT == 1)

#else
#if (SSP_PORT == 2)
#define SDCMR_RX DRCMR15
#define SDCMR_TX DRCMR16
#else
#if (SSP_PORT == 3)
#define SDCMR_RX DRCMR66
#define SDCMR_TX DRCMR67
#else
#if (SSP_PORT == 4)
#define SDCMR_RX DRCMRRXSADR
#define SDCMR_TX DRCMRTXSADR
#endif
#endif
#endif
#endif
#else 


#endif


#ifdef PXA_310_LV
#define CLOCK_FACTOR 2

#else 
#define CLOCK_FACTOR 2
#endif


#define CLOCK_DIVIDER(i)((i-1)<<8)	


#define DMA_INT_MASK (DCSR_ENDINTR | DCSR_STARTINTR | DCSR_BUSERR)
#define RESET_DMA_CHANNEL (DCSR_NODESC | DMA_INT_MASK)

#define SSP_TIMEOUT_SCALE (769)
#define SSP_TIMEOUT(x) ((x*10000)/SSP_TIMEOUT_SCALE)

#define SPI_PACKET_SIZE 256



#if 0
// in android platform 2.6.25 , need to check the Reg bit by bit later
#define GSDR(x) __REG2(0x40e00400, ((x) & 0x60) >> 3)
#define GCDR(x) __REG2(0x40300420, ((x) & 0x60) >> 3)

#define GSRER(x) __REG2(0x40e00440, ((x) & 0x60) >> 3)
#define GCRER(x) __REG2(0x40e00460, ((x) & 0x60) >> 3)
#endif

#define GPIO_DIR_IN 0
#if 0
#define SSCR0_P1	__REG(0x41000000)  /* SSP Port 1 Control Register 0 */
#define SSCR1_P1	__REG(0x41000004)  /* SSP Port 1 Control Register 1 */
#define SSSR_P1		__REG(0x41000008)  /* SSP Port 1 Status Register */
#define SSITR_P1	__REG(0x4100000C)  /* SSP Port 1 Interrupt Test Register */
#define SSDR_P1		__REG(0x41000010)  /* (Write / Read) SSP Port 1 Data Write Register/SSP Data Read Register */



#if 1
/* PXA27x ports */
#if defined (CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
#define SSTO_P1		__REG(0x41000028)  /* SSP Port 1 Time Out Register */
#define SSPSP_P1	__REG(0x4100002C)  /* SSP Port 1 Programmable Serial Protocol */
#define SSTSA_P1	__REG(0x41000030)  /* SSP Port 1 Tx Timeslot Active */
#define SSRSA_P1	__REG(0x41000034)  /* SSP Port 1 Rx Timeslot Active */
#define SSTSS_P1	__REG(0x41000038)  /* SSP Port 1 Timeslot Status */
#define SSACD_P1	__REG(0x4100003C)  /* SSP Port 1 Audio Clock Divider */
#define SSCR0_P2	__REG(0x41700000)  /* SSP Port 2 Control Register 0 */
#define SSCR1_P2	__REG(0x41700004)  /* SSP Port 2 Control Register 1 */
#define SSSR_P2		__REG(0x41700008)  /* SSP Port 2 Status Register */
#define SSITR_P2	__REG(0x4170000C)  /* SSP Port 2 Interrupt Test Register */
#define SSDR_P2		__REG(0x41700010)  /* (Write / Read) SSP Port 2 Data Write Register/SSP Data Read Register */
#define SSTO_P2		__REG(0x41700028)  /* SSP Port 2 Time Out Register */
#define SSPSP_P2	__REG(0x4170002C)  /* SSP Port 2 Programmable Serial Protocol */
#define SSTSA_P2	__REG(0x41700030)  /* SSP Port 2 Tx Timeslot Active */
#define SSRSA_P2	__REG(0x41700034)  /* SSP Port 2 Rx Timeslot Active */
#define SSTSS_P2	__REG(0x41700038)  /* SSP Port 2 Timeslot Status */
#define SSACD_P2	__REG(0x4170003C)  /* SSP Port 2 Audio Clock Divider */
#define	SSACDD_P2	__REG(0x41700040)  /* SSP Port 2 Audio Clock Dither Divider Register */

#define SSCR0_P3	__REG(0x41900000)  /* SSP Port 3 Control Register 0 */
#define SSCR1_P3	__REG(0x41900004)  /* SSP Port 3 Control Register 1 */
#define SSSR_P3		__REG(0x41900008)  /* SSP Port 3 Status Register */
#define SSITR_P3	__REG(0x4190000C)  /* SSP Port 3 Interrupt Test Register */
#define SSDR_P3		__REG(0x41900010)  /* (Write / Read) SSP Port 3 Data Write Register/SSP Data Read Register */
#define SSTO_P3		__REG(0x41900028)  /* SSP Port 3 Time Out Register */
#define SSPSP_P3	__REG(0x4190002C)  /* SSP Port 3 Programmable Serial Protocol */
#define SSTSA_P3	__REG(0x41900030)  /* SSP Port 3 Tx Timeslot Active */
#define SSRSA_P3	__REG(0x41900034)  /* SSP Port 3 Rx Timeslot Active */
#define SSTSS_P3	__REG(0x41900038)  /* SSP Port 3 Timeslot Status */
#define SSACD_P3	__REG(0x4190003C)  /* SSP Port 3 Audio Clock Divider */
#define	SSACDD_P3	__REG(0x41900040)  /* SSP Port 3 Audio Clock Dither Divider Register */

#define SSCR0_P4        __REG(0x41A00000)  /* SSP Port 4 Control Register 0 */
#define SSCR1_P4        __REG(0x41A00004)  /* SSP Port 4 Control Register 1 */
#define SSSR_P4         __REG(0x41A00008)  /* SSP Port 4 Status Register */
#define SSITR_P4        __REG(0x41A0000C)  /* SSP Port 4 Interrupt Test Register */
#define SSDR_P4         __REG(0x41A00010)  /* (Write / Read) SSP Port 4 Data Write Register/SSP Data Read Register */
#define SSTO_P4         __REG(0x41A00028)  /* SSP Port 4 Time Out Register */
#define SSPSP_P4        __REG(0x41A0002C)  /* SSP Port 4 Programmable Serial Protocol */
#define	SSTSA_P4	__REG(0x41A00030)  /* SSP Port 4 TX Time Slot Active Register */
#define	SSRSA_P4	__REG(0x41A00034)  /* SSP Port 4 RX Time Slot Active Register */
#define	SSTSS_P4	__REG(0x41A00038)  /* SSP Port 4 Time Slot Status Register */
#define	SSACD_P4	__REG(0x41A0003C)  /* SSP Port 4 Audio Clock Divider Register */
#define	SSACDD_P4	__REG(0x41A00040)  /* SSP Port 4 Audio Clock Dither Divider Register */

#else /* PXA255 (only port 2) and PXA26x ports*/
#define SSTO_P1		__REG(0x41000028)  /* SSP Port 1 Time Out Register */
#define SSPSP_P1	__REG(0x4100002C)  /* SSP Port 1 Programmable Serial Protocol */
#define SSCR0_P2	__REG(0x41400000)  /* SSP Port 2 Control Register 0 */
#define SSCR1_P2	__REG(0x41400004)  /* SSP Port 2 Control Register 1 */
#define SSSR_P2		__REG(0x41400008)  /* SSP Port 2 Status Register */
#define SSITR_P2	__REG(0x4140000C)  /* SSP Port 2 Interrupt Test Register */
#define SSDR_P2		__REG(0x41400010)  /* (Write / Read) SSP Port 2 Data Write Register/SSP Data Read Register */
#define SSTO_P2		__REG(0x41400028)  /* SSP Port 2 Time Out Register */
#define SSPSP_P2	__REG(0x4140002C)  /* SSP Port 2 Programmable Serial Protocol */
#define SSCR0_P3	__REG(0x41500000)  /* SSP Port 3 Control Register 0 */
#define SSCR1_P3	__REG(0x41500004)  /* SSP Port 3 Control Register 1 */
#define SSSR_P3		__REG(0x41500008)  /* SSP Port 3 Status Register */
#define SSITR_P3	__REG(0x4150000C)  /* SSP Port 3 Interrupt Test Register */
#define SSDR_P3		__REG(0x41500010)  /* (Write / Read) SSP Port 3 Data Write Register/SSP Data Read Register */
#define SSTO_P3		__REG(0x41500028)  /* SSP Port 3 Time Out Register */
#define SSPSP_P3	__REG(0x4150002C)  /* SSP Port 3 Programmable Serial Protocol */
#endif


#define SSCR0_P(x) (*(((x) == 1) ? &SSCR0_P1 : ((x) == 2) ? &SSCR0_P2 : ((x) == 3) ? &SSCR0_P3 : ((x) == 4) ? &SSCR0_P4 : NULL))
#define SSCR1_P(x) (*(((x) == 1) ? &SSCR1_P1 : ((x) == 2) ? &SSCR1_P2 : ((x) == 3) ? &SSCR1_P3 : ((x) == 4) ? &SSCR1_P4 : NULL))
#define SSSR_P(x) (*(((x) == 1) ? &SSSR_P1 : ((x) == 2) ? &SSSR_P2 : ((x) == 3) ? &SSSR_P3 : ((x) == 4) ? &SSSR_P4 : NULL))
#define SSITR_P(x) (*(((x) == 1) ? &SSITR_P1 : ((x) == 2) ? &SSITR_P2 : ((x) == 3) ? &SSITR_P3 : ((x) == 4) ? &SSITR_P4 : NULL))
#define SSDR_P(x) (*(((x) == 1) ? &SSDR_P1 : ((x) == 2) ? &SSDR_P2 : ((x) == 3) ? &SSDR_P3 : ((x) == 4) ? &SSDR_P4 : NULL))
#define SSTO_P(x) (*(((x) == 1) ? &SSTO_P1 : ((x) == 2) ? &SSTO_P2 : ((x) == 3) ? &SSTO_P3 : ((x) == 4) ? &SSTO_P4 : NULL))
#define SSPSP_P(x) (*(((x) == 1) ? &SSPSP_P1 : ((x) == 2) ? &SSPSP_P2 : ((x) == 3) ? &SSPSP_P3 : ((x) == 4) ? &SSPSP_P4 : NULL))
#define SSTSA_P(x) (*(((x) == 1) ? &SSTSA_P1 : ((x) == 2) ? &SSTSA_P2 : ((x) == 3) ? &SSTSA_P3 : ((x) == 4) ? &SSTSA_P4 : NULL))
#define SSRSA_P(x) (*(((x) == 1) ? &SSRSA_P1 : ((x) == 2) ? &SSRSA_P2 : ((x) == 3) ? &SSRSA_P3 : ((x) == 4) ? &SSRSA_P4 : NULL))
#define SSTSS_P(x) (*(((x) == 1) ? &SSTSS_P1 : ((x) == 2) ? &SSTSS_P2 : ((x) == 3) ? &SSTSS_P3 : ((x) == 4) ? &SSTSS_P4 : NULL))
#define SSACD_P(x) (*(((x) == 1) ? &SSACD_P1 : ((x) == 2) ? &SSACD_P2 : ((x) == 3) ? &SSACD_P3 : ((x) == 4) ? &SSACD_P4 : NULL))
#define SSACDD_P(x) (*(((x) == 2) ? &SSACDD_P2 : ((x) == 3) ? &SSACDD_P3 : ((x) == 4) ? &SSACDD_P4 : NULL))
#endif
#endif
unsigned  long u_irq_count =0;

char  SMS_IRQ_GPIO=MFP_PIN_GPIO14;




#define DMA_BURST_SIZE  	DCMD_BURST16      
#define SPI_RX_FIFO_RFT 	SSCR1_RxTresh(4) 
#define SPI_TX_FIFO_TFT		SSCR1_TxTresh(3) 


/**********************************************************************/

#include <linux/notifier.h>

static unsigned int dma_rxbuf_phy_addr ;
static unsigned int dma_txbuf_phy_addr ;

static int     rx_dma_channel =0 ;
static int     tx_dma_channel =0 ;
static volatile int     dma_len = 0 ;
static volatile int     tx_len  = 0 ;

static struct ssp_dev*  panic_sspdev = NULL ;


extern void spilog_panic_print(void) ;
static void chip_powerdown(void);
extern void smschar_reset_device(void);



void pxa3xx_gpio_set_rising_edge_detect (int gpio_id, int dir)
{
	unsigned  long flags;
 	int gpio = mfp_to_gpio(gpio_id);


	local_irq_save(flags);
        
        if ( dir == GPIO_DIR_IN)
		GCRER(gpio) =1u << (gpio& 0x1f);
	else
		GSRER(gpio) =1u << (gpio& 0x1f);
 	local_irq_restore(flags);


}

void pxa3xx_gpio_set_direction(int gpio_id , int dir)
{
	unsigned long flags;
	int gpio = mfp_to_gpio(gpio_id);

	local_irq_save(flags);
	
        if ( dir == GPIO_DIR_IN)
		GCDR(gpio) =1u << (gpio& 0x1f);
	else
		GSDR(gpio) =1u << (gpio& 0x1f);
 	local_irq_restore(flags);

}

struct spiphy_dev_s {
	struct ssp_dev sspdev;	
	struct completion transfer_in_process;
	void (*interruptHandler) (void *);
	void *intr_context;
	struct device *dev;	
	int rx_dma_channel;
	int tx_dma_channel;
	int rx_buf_len;
	int tx_buf_len;
};





static inline u32 invert_bo(u32 u)
{
	return ((u & 0xff) << 24) | ((u & 0xff00) << 8) | ((u & 0xff0000) >> 8)
		| ((u & 0xff000000) >> 24);
}



static int invert_endianness(char *buf, int len)
{
	int i;
	u32 *ptr = (u32 *) buf;

	len = (len + 3) / 4;
	for (i = 0; i < len; i++, ptr++)
		*ptr = invert_bo(*ptr);

	return 4 * ((len + 3) & (~3));
}


static unsigned long dma_map_buf(struct spiphy_dev_s *spiphy_dev, char *buf,
		int len, int direction)
{
	unsigned long phyaddr;
	
	if (!buf) {
		PERROR(" NULL buffers to map\n");
		return 0;
	}
	
	phyaddr = dma_map_single(spiphy_dev->dev, buf, len, direction);
	if (dma_mapping_error(spiphy_dev->dev, phyaddr)) {
		PERROR("exiting  with error\n");
		return 0;
	}
	return phyaddr;
}

static irqreturn_t spibus_interrupt(int irq, void *context)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
	u_irq_count ++;


	if (spiphy_dev->interruptHandler)
		spiphy_dev->interruptHandler(spiphy_dev->intr_context);
	return IRQ_HANDLED;

}



static void spibus_dma_handler(int channel, void *context)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
	u32 irq_status = DCSR(channel) & DMA_INT_MASK;


	if (irq_status & DCSR_BUSERR) {
		printk(KERN_EMERG "bus error!!! resetting channel %d\n", channel);

		DCSR(spiphy_dev->rx_dma_channel) = RESET_DMA_CHANNEL;
		DCSR(spiphy_dev->tx_dma_channel) = RESET_DMA_CHANNEL;
	}
	DCSR(spiphy_dev->rx_dma_channel) = RESET_DMA_CHANNEL;
	complete(&spiphy_dev->transfer_in_process);
}

void smsspibus_xfer(void *context, unsigned char *txbuf,
		    unsigned long txbuf_phy_addr, unsigned char *rxbuf,
		    unsigned long rxbuf_phy_addr, int len)
{

	unsigned long tmp[4];


	unsigned long txdma;
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
        unsigned long expire;
	int res =0;

        expire =  msecs_to_jiffies(200) ;
	
	if (txbuf)
	{

		invert_endianness(txbuf, len);
	}

	tmp[0] = -1;
	tmp[1] = -1;
	tmp[2] = -1;
	tmp[3] = -1;



	if (!txbuf)
        {
		txdma =
		    dma_map_buf(spiphy_dev, (char *)tmp, sizeof(tmp),
				DMA_TO_DEVICE);
                tx_len = 0 ;
        }
	else
        {
		txdma = txbuf_phy_addr;
                tx_len = len ;
        }

	init_completion(&spiphy_dev->transfer_in_process);

	DCSR(spiphy_dev->rx_dma_channel) = RESET_DMA_CHANNEL;
	DSADR(spiphy_dev->rx_dma_channel) = SSP_FIFO;
		DTADR(spiphy_dev->rx_dma_channel) = rxbuf_phy_addr;


     
	DCMD(spiphy_dev->rx_dma_channel) = DCMD_INCTRGADDR | DCMD_FLOWSRC
	    | DCMD_WIDTH4 | DCMD_ENDIRQEN | DMA_BURST_SIZE | len;



        rx_dma_channel = spiphy_dev->rx_dma_channel ;
        dma_rxbuf_phy_addr = (unsigned int) rxbuf_phy_addr ;
        dma_len = len ;

	spiphy_dev->rx_buf_len = len;

	DCSR(spiphy_dev->tx_dma_channel) = RESET_DMA_CHANNEL;
	DTADR(spiphy_dev->tx_dma_channel) = SSP_FIFO;     
      
	DSADR(spiphy_dev->tx_dma_channel) = txdma;
       
        tx_dma_channel = spiphy_dev->tx_dma_channel;
        dma_txbuf_phy_addr = (unsigned int) txdma ;

	if (txbuf) {
		DCMD(spiphy_dev->tx_dma_channel) =
		    DCMD_INCSRCADDR | DCMD_FLOWTRG | DCMD_WIDTH4
		   | DMA_BURST_SIZE  | len;
		spiphy_dev->tx_buf_len = len;
	} else {
		DCMD(spiphy_dev->tx_dma_channel) = DCMD_FLOWTRG
		    | DCMD_WIDTH4   |DMA_BURST_SIZE  | len;
		spiphy_dev->tx_buf_len = 4;
	}

#if 0
       printk("Tx DSADR=[0x%x],DTADR=[0x%x],DCMD=[0x%x],len =[0x%x]\n",
                DSADR(spiphy_dev->tx_dma_channel),DTADR(spiphy_dev->tx_dma_channel),DCMD(spiphy_dev->tx_dma_channel),len) ;
#endif 

	if (rxbuf_phy_addr & 0x7)
		DALGN |= (1 << spiphy_dev->rx_dma_channel);
	else
		DALGN &= ~(1 << spiphy_dev->rx_dma_channel);
	if (txdma & 0x7)
		DALGN |= (1 << spiphy_dev->tx_dma_channel);
	else
		DALGN &= ~(1 << spiphy_dev->tx_dma_channel);


  
	DCSR(spiphy_dev->rx_dma_channel) |= DCSR_RUN;
	DCSR(spiphy_dev->tx_dma_channel) |= DCSR_RUN;

	res = wait_for_completion_timeout(&spiphy_dev->transfer_in_process,expire);       
 	if(!res)
	{
             printk( "smsmdtv DMA timeout, res=0x%x len =%d\n", res, len);
             printk( "smsmdtv DMA reg, 0x%x %x \n",DCSR(spiphy_dev->rx_dma_channel), DCSR(spiphy_dev->tx_dma_channel)  );
             DCSR(spiphy_dev->rx_dma_channel) = RESET_DMA_CHANNEL;
	     DCSR(spiphy_dev->tx_dma_channel) = RESET_DMA_CHANNEL;
	     
             complete(&spiphy_dev->transfer_in_process);
        }
    	{
		invert_endianness(rxbuf, len);

		if (!txbuf)
			PDEBUG("rx[4]:0x%x;[6]:0x%x \n", rxbuf[4], rxbuf[6]);
	}


}

void smschipreset(void *context)
{

}

static struct ssp_state  sms_ssp_state ;

void smsspibus_ssp_suspend(void* context )
{
    struct spiphy_dev_s *spiphy_dev ;
    printk("entering smsspibus_ssp_suspend\n");
    if(!context)
    {
        PERROR("smsspibus_ssp_suspend context NULL \n") ;
        return ;
    }
    spiphy_dev = (struct spiphy_dev_s *) context;

    ssp_flush(&(spiphy_dev->sspdev)) ;
    ssp_save_state(&(spiphy_dev->sspdev) , &sms_ssp_state) ;
    ssp_disable(&(spiphy_dev->sspdev));
    ssp_exit(&spiphy_dev->sspdev);
    free_irq(IRQ_GPIO(mfp_to_gpio(SMS_IRQ_GPIO)), spiphy_dev);

    	
    if (spiphy_dev->rx_dma_channel >= 0)
 	pxa_free_dma(spiphy_dev->rx_dma_channel);

    if (spiphy_dev->tx_dma_channel >= 0)
	pxa_free_dma(spiphy_dev->tx_dma_channel);
	chip_powerdown();
    
}
static void chip_poweron(void)
{
	int err = 0;
	int boardid = pm860x_get_boardID();

#ifdef CONFIG_MACH_LC6830_PHONE_BOARD_1_0
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO4), 1);
	mdelay(100);
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO6), 1);
	mdelay(1);
#else 
        
	
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO15), "cmmb-enable");
        if (err) {
                printk("failed to request GPIO15 for cmmb-enable\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO15), 1);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO15));
	mdelay(10);
	
	#ifdef CONFIG_PXA_U880
        printk("U880\n");
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO124), "cmmb-pwd");
        if (err) {
                printk("failed to request GPIO124 for cmmb-enable\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO124), 1);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO124));
	mdelay(150);
	#elif CONFIG_PXA_U812
        printk("U812\n");
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO4), "cmmb-pwd");
        if (err) {
                printk("failed to request GPIO4 for cmmb-enable\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO4), 1);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO4));
	mdelay(150);
	#else
	printk("U810/U802\n");

    
    	#ifdef CONFIG_PXA_U802
   	if(boardid == ZTE_HWVERSION2)
    	{
                err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO7), "cmmb-pwd");
        	if (err){
                	printk("failed to request GPIO7 for cmmb-enable\n");
                	return;
        	}
        	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO7), 1);
        	gpio_free(mfp_to_gpio(MFP_PIN_GPIO7));
		mdelay(150);
    	}
	else
   	#endif
    	
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO145), "cmmb-pwd");
        if (err) {
                printk("failed to request GPIO145 for cmmb-enable\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO145), 1);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO145));
	mdelay(150);
	#endif
	
#endif
}

static void chip_powerdown(void)
{
	int err = 0;
	int boardid = pm860x_get_boardID();

#ifdef CONFIG_MACH_LC6830_PHONE_BOARD_1_0
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO4), 0);
	mdelay(50);
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO6), 0);
#else 
        
	
	
	
	
	#ifdef CONFIG_PXA_U880
	printk("U880\n");
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO124), "cmmb-pwd");
        if (err) {
                printk("failed to request GPIO124 for cmmb-enable\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO124), 0);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO124));
	#elif CONFIG_PXA_U812
        printk("U812\n");
	err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO4), "cmmb-pwd");
        if (err) {
                printk("failed to request GPIO4 for cmmb-enable\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO4), 0);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO4));
	#else
	printk("U810/U802\n");

    	
    	#ifdef CONFIG_PXA_U802
   	if(boardid == ZTE_HWVERSION2)
    	{
        	err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO7), "cmmb-pwd");
        	if (err) {
                	printk("failed to request GPIO7 for cmmb-enable\n");
                	return;
       		 }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO7), 0);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO7));
    	}
	else
   	#endif
    	
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO145), "cmmb-pwd");
        if (err) {
                printk("failed to request GPIO145 for cmmb-enable\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO145), 0);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO145));
	#endif
	
	
	
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO15), "cmmb-enable");
        if (err) {
                printk("failed to request GPIO15 for cmmb-enable\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO15), 0);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO15));
	
#endif
}

int smsspibus_ssp_resume(void* context) 
{
    int ret;
    struct spiphy_dev_s *spiphy_dev ;
    u32 mode = 0, flags = 0, psp_flags = 0, speed = 0;
    int boardid = pm860x_get_boardID();
    printk("entering smsspibus_ssp_resume\n");

    if(!context)
    {
        PERROR("smsspibus_ssp_resume context NULL \n");
        return -1;
    }
    spiphy_dev = (struct spiphy_dev_s *) context;
    #ifdef CONFIG_PXA_U802
    if(boardid == ZTE_HWVERSION2)
    {
        SMS_IRQ_GPIO=MFP_PIN_GPIO5;
    }
    #endif
    chip_poweron();
    ret = ssp_init(&spiphy_dev->sspdev, SSP_PORT, 0);
	if (ret) {
		PERROR("ssp_init failed. error %d", ret);
	}
    speed = CLOCK_DIVIDER(CLOCK_FACTOR); 
   
    mode = SSCR0_Motorola | SSCR0_DataSize(16) | SSCR0_EDSS;
   
    flags = SPI_RX_FIFO_RFT |SPI_TX_FIFO_TFT | SSCR1_TSRE |
    SSCR1_RSRE | SSCR1_RIE | SSCR1_TRAIL;	
    ssp_config(&spiphy_dev->sspdev, mode, flags, psp_flags, speed);
  

	pxa3xx_gpio_set_rising_edge_detect(SMS_IRQ_GPIO, 1);
	pxa3xx_gpio_set_direction(SMS_IRQ_GPIO, GPIO_DIR_IN);

    	
    spiphy_dev->rx_dma_channel =
               pxa_request_dma("spibusdrv_rx", DMA_PRIO_MEDIUM, spibus_dma_handler,
			    spiphy_dev);
    if (spiphy_dev->rx_dma_channel < 0) {
	ret = -EBUSY;
	printk(KERN_EMERG "Could not get RX DMA channel.\n");
	goto error_rxdma;
    }
    printk("rx dma channel:%d\n",spiphy_dev->rx_dma_channel);
    spiphy_dev->tx_dma_channel =
               pxa_request_dma("spibusdrv_tx", DMA_PRIO_MEDIUM, spibus_dma_handler,
			    spiphy_dev);
    if (spiphy_dev->tx_dma_channel < 0) {
	ret = -EBUSY;
	printk(KERN_EMERG "Could not get TX DMA channel.\n");
	goto error_txdma;
    }	
    printk("tx dma channel:%d\n",spiphy_dev->tx_dma_channel);
	 DRCMR(52) = DRCMR_MAPVLD | spiphy_dev->rx_dma_channel;
	 DRCMR(53) = DRCMR_MAPVLD | spiphy_dev->tx_dma_channel;

    set_irq_type(IRQ_GPIO(mfp_to_gpio(SMS_IRQ_GPIO)), IRQ_TYPE_EDGE_RISING);
    ret = request_irq(IRQ_GPIO(mfp_to_gpio(SMS_IRQ_GPIO)), spibus_interrupt,
			IRQF_DISABLED, "SMSSPI", spiphy_dev);                             
		printk("interrupt for SMS device. status =%d\n",ret);
    if (ret) {
		printk("Could not get interrupt for SMS device. status =%d\n",
		       ret);
		goto error_irq;
	}
	printk("irq level:%d\n",gpio_get_value(mfp_to_gpio(SMS_IRQ_GPIO)));

    ssp_restore_state(&(spiphy_dev->sspdev), &sms_ssp_state) ;

    return 0 ;
error_irq:
	if (spiphy_dev->tx_dma_channel >= 0)
		pxa_free_dma(spiphy_dev->tx_dma_channel);
error_txdma:
	if (spiphy_dev->rx_dma_channel >= 0)
		pxa_free_dma(spiphy_dev->rx_dma_channel);

error_rxdma:
	ssp_exit(&spiphy_dev->sspdev);
    return -1 ;

}


void *smsspiphy_init(void *context, void (*smsspi_interruptHandler) (void *),
		     void *intr_context)
{
	
	struct spiphy_dev_s *spiphy_dev;
	
	PDEBUG("entering\n");
	spiphy_dev = kmalloc(sizeof(struct spiphy_dev_s), GFP_KERNEL);
        if(!spiphy_dev )
        {
          printk("spiphy_dev is null in smsspiphy_init\n") ;
          return NULL;
	}
	chip_powerdown();
	spiphy_dev->interruptHandler = smsspi_interruptHandler;
	spiphy_dev->intr_context = intr_context;
#ifdef PXA_310_LV
                          
#else 
	set_irq_type(IRQ_GPIO(22), IRQT_FALLING);
	ret =
	    request_irq(IRQ_GPIO(22), spibus_interrupt, SA_INTERRUPT, "SMSSPI",
			&(g_spidata.sspdev));
#endif

        
        panic_sspdev =  &(spiphy_dev->sspdev) ;
        
	PDEBUG("exiting\n");
	return spiphy_dev;

}

int smsspiphy_deinit(void *context)
{
	
	PDEBUG("entering\n");

	printk("entering smsspiphy_deinit\n");

        panic_sspdev = NULL;
    
        chip_powerdown();
	PDEBUG("exiting\n");
	return 0;
}

void smsspiphy_set_config(struct spiphy_dev_s *spiphy_dev, int clock_divider)
{
	u32 mode, flags, speed, psp_flags = 0;
	ssp_disable(&spiphy_dev->sspdev);
	
	speed = CLOCK_DIVIDER(clock_divider);
	
	mode = SSCR0_Motorola | SSCR0_DataSize(16) | SSCR0_EDSS;
	flags = SPI_RX_FIFO_RFT |SPI_TX_FIFO_TFT | SSCR1_TSRE |
		 SSCR1_RSRE | SSCR1_RIE | SSCR1_TRAIL;	
	ssp_config(&spiphy_dev->sspdev, mode, flags, psp_flags, speed);
	ssp_enable(&spiphy_dev->sspdev);
}

void prepareForFWDnl(void *context)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
	smsspiphy_set_config(spiphy_dev, 3);
	msleep(100);
}

void fwDnlComplete(void *context, int App)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
	smsspiphy_set_config(spiphy_dev, 1);
	msleep(100);
}
