#include "yl_mmu_rcache.h"

#include "yl_mmu_kernel_alloc.h"


void 
Rcache_Init( Rcache self )
{
    int i;

    yl_mmu_zeromem( (void *)self, sizeof(RcacheRec) );

    for ( i=1; i<REMAPPER_RCACHE_ITEM_LENGTH; ++i )
    {
        //self->items[ i ].q_prev = (Rcache_link_t)( i-1 );
        self->items[ i ].q_next = (Rcache_link_t)( i+1 );
    }

    self->empty_queue.head                              = (Rcache_link_t)1;
    self->empty_queue.tail                              = (Rcache_link_t)(REMAPPER_RCACHE_ITEM_LENGTH-1);
    self->items[REMAPPER_RCACHE_ITEM_LENGTH-1].q_next   = (Rcache_link_t)0; 

    self->used_count = 0;
}

void 
Rcache_Uninit( Rcache self )
{
    yl_mmu_zeromem( (void *)self, sizeof(RcacheRec) );
}

