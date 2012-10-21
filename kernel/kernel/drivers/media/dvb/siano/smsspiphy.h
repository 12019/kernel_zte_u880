

#ifndef __SMS_SPI_PHY_H__
#define __SMS_SPI_PHY_H__

void smsspibus_xfer(void *context, unsigned char *txbuf,
		    unsigned long txbuf_phy_addr, unsigned char *rxbuf,
		    unsigned long rxbuf_phy_addr, int len);
void *smsspiphy_init(void *context, void (*smsspi_interruptHandler) (void *),
		     void *intr_context);
void smsspiphy_deinit(void *context);
void smschipreset(void *context);
void WriteFWtoStellar(void *pSpiPhy, unsigned char *pFW, unsigned long Len);
void prepareForFWDnl(void *pSpiPhy);
void fwDnlComplete(void *context, int App);
void smsspibus_ssp_suspend(void* context );
int  smsspibus_ssp_resume(void* context);

#endif 
