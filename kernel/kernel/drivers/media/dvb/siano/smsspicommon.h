
#ifndef _SMS_SPI_COMMON_H_
#define _SMS_SPI_COMMON_H_

#define RX_PACKET_SIZE  		0x1000
#define SPI_PACKET_SIZE_BITS		8
#define SPI_PACKET_SIZE 		(1<<SPI_PACKET_SIZE_BITS)
#define SPI_MAX_CTRL_MSG_SIZE		0x100

#define MSG_HDR_FLAG_SPLIT_MSG_HDR	0x0004
#define MSG_HDR_LEN			8

enum _spi_rx_state {
	RxsWait_a5 = 0,
	RxsWait_5a,
	RxsWait_e7,
	RxsWait_7e,
	RxsTypeH,
	RxsTypeL,
	RxsGetSrcId,
	RxsGetDstId,
	RxsGetLenL,
	RxsGetLenH,
	RxsFlagsL,
	RxsFlagsH,
	RxsData
};

struct _rx_buffer_st {
	void *ptr;
	unsigned long phy_addr;
};

struct _rx_packet_request {
	struct _rx_buffer_st *msg_buf;
	int msg_offset;
	int msg_len;
	int msg_flags;
};

struct _spi_dev_cb_st{
	void (*transfer_data_cb) (void *context, unsigned char *, unsigned long,
				  unsigned char *, unsigned long, int);
	void (*msg_found_cb) (void *, void *, int, int);
	struct _rx_buffer_st *(*allocate_rx_buf) (void *, int);
	void (*free_rx_buf) (void *, struct _rx_buffer_st *);
};

struct _spi_dev {
	void *context;
	void *phy_context;
	struct _spi_dev_cb_st cb;
	char *rxbuf;
	enum _spi_rx_state rxState;
	struct _rx_packet_request rxPacket;
	char *internal_tx_buf;
};

struct _spi_msg {
	char *buf;
	unsigned long buf_phy_addr;
	int len;
};

void smsspi_common_transfer_msg(struct _spi_dev *dev, struct _spi_msg *txmsg,
				int padding_allowed);
int smsspicommon_init(struct _spi_dev *dev, void *contex, void *phy_context,
		      struct _spi_dev_cb_st *cb);

#if defined HEXDUMP_DEBUG && defined SPIBUS_DEBUG

void smsspi_khexdump(char *buf, int len);
#else
#define smsspi_khexdump(buf, len)
#endif

#endif 
