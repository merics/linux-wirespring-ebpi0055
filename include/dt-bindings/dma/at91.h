/*
 * This header provides macros for at91 dma bindings.
 *
 * Copyright (C) 2013 Ludovic Desroches <ludovic.desroches@atmel.com>
 *
 * GPLv2 only
 */

#ifndef __DT_BINDINGS_AT91_DMA_H__
#define __DT_BINDINGS_AT91_DMA_H__

/*
 * Source and/or destination peripheral ID
 */
#define AT91_DMA_CFG_PER_ID_MASK	(0xff)
#define AT91_DMA_CFG_PER_ID(id)		(id & AT91_DMA_CFG_PER_ID_MASK)

/*
 * FIFO configuration: it defines when a request is serviced.
 */
#define AT91_DMA_CFG_FIFOCFG_OFFSET	(8)
#define AT91_DMA_CFG_FIFOCFG_MASK	(0xf << AT91_DMA_CFG_FIFOCFG_OFFSET)
#define AT91_DMA_CFG_FIFOCFG_HALF	(0x0 << AT91_DMA_CFG_FIFOCFG_OFFSET)	/* half FIFO (default behavior) */
#define AT91_DMA_CFG_FIFOCFG_ALAP	(0x1 << AT91_DMA_CFG_FIFOCFG_OFFSET)	/* largest defined AHB burst */
#define AT91_DMA_CFG_FIFOCFG_ASAP	(0x2 << AT91_DMA_CFG_FIFOCFG_OFFSET)	/* single AHB access */

#endif /* __DT_BINDINGS_AT91_DMA_H__ */
