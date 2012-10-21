

#ifndef _SMS_DBG_H_
#define _SMS_DBG_H_

#include <linux/kernel.h>
#include <linux/module.h>


#undef PERROR
#  define PERROR(fmt, args...) \
	printk(KERN_ERR "spibus error: line %d- %s(): " fmt, __LINE__,\
	  __func__, ## args)
#undef PWARNING
#  define PWARNING(fmt, args...) \
	printk(KERN_WARNING "spibus warning: line %d- %s(): " fmt, __LINE__,  \
	__func__, ## args)






#undef PDEBUG			
#ifdef SPIBUS_DEBUG

#define PDEBUG(fmt, args...) \
	printk(KERN_DEBUG " " fmt,## args)

#else
#  define PDEBUG(fmt, args...)	
#endif


#define TXT(str) str
#define PRN_DBG(str) PDEBUG str
#define PRN_ERR(str) PERROR str

#endif 
