//#define QE_BASE 0x100000	// for e300 it is 0x100000, for e500 it is 0x80000
//#define IMMRBAR 0xe0000000	// usually it is 0xe0000000

#define IMMRBAR (void*)qe_immr		// Use the global variable instead of hard coded address 0xe0000000
#define QE_BASE 0			// qe_immr already includes the offset to the QE registers

#define QE_REV  2			// define the qe revision number (1 or 2)
//#define PRINT   ________  // define the print function (printf or XX_Print ...)
#define PRINT(format, args...) \
		printk(KERN_DEBUG"QE: "format,##args)

#define TCRL_PM 0x01010460  // program TCRL for default PM settings

//#include <stdio.h>
#include <linux/kernel.h>
#include <asm/div64.h>
#include <asm/immap_qe.h>

void enable_trb_r1(void)
{

unsigned int *param_ptr;
unsigned int  tcrl_default = 0;

    if ( 1 )
      tcrl_default = TCRL_PM;

    param_ptr = (unsigned int *)(IMMRBAR + QE_BASE + 0x4600); // TRB1

    *param_ptr   = 0x00004000; // Disable TCRH + reset counters
    *param_ptr++ = 0x00000000;
    *param_ptr++ = 0x00000000 | tcrl_default; // TCRL
    *param_ptr++ = 0xFFFFFFFF; // TESR: reset	

    *param_ptr++ = 0xffffff00; // TECR0H
    *param_ptr++ = 0xa0c00000; // TECR0L
    *param_ptr++ = 0x00000020; // TERF0H
    *param_ptr++ = 0x00000000; // TERF0L
    *param_ptr++ = 0x00000000; // TECR1H
    *param_ptr++ = 0x00000000; // TECR1L
    *param_ptr++ = 0x00000000; // TERF1H
    *param_ptr++ = 0x00000000; // TERF1L
    param_ptr = (unsigned int *)(IMMRBAR + QE_BASE + 0x4654); // move to TSNUM1 offset
    *param_ptr++ = 0x00000000; // TSNUM1
    *param_ptr++ = 0x80000000; // TSNUM2
    *param_ptr++ = 0x00000000; // TSNUM3
    *param_ptr++ = 0x00000000; // TSNUM4

    //BPRMTR 
    param_ptr = (unsigned int *)(IMMRBAR + QE_BASE + 0x409c);
    *param_ptr = 0x00000000; // BPRMTR

    //TCRH 
    param_ptr = (unsigned int *)(IMMRBAR + QE_BASE + 0x4600); // TRB1
    *param_ptr = 0xcc002000; // TCRH
}


void enable_trb_r2(void)
{

unsigned int *param_ptr;
unsigned int  tcrl_default = 0;

    if ( 1 )
      tcrl_default = TCRL_PM;

    param_ptr = (unsigned int *)(IMMRBAR + QE_BASE + 0x4680); // TRB2

    *param_ptr   = 0x00004000; // Disable TCRH + reset counters
    *param_ptr++ = 0x00000000;
    *param_ptr++ = 0x00000000 | tcrl_default; // TCRL
    *param_ptr++ = 0xFFFFFFFF; // TESR: reset	

    *param_ptr++ = 0xffffff00; // TECR0H
    *param_ptr++ = 0xa0c00000; // TECR0L
    *param_ptr++ = 0x00000020; // TERF0H
    *param_ptr++ = 0x00000000; // TERF0L
    *param_ptr++ = 0x00000000; // TECR1H
    *param_ptr++ = 0x00000000; // TECR1L
    *param_ptr++ = 0x00000000; // TERF1H
    *param_ptr++ = 0x00000000; // TERF1L
    param_ptr = (unsigned int *)(IMMRBAR + QE_BASE + 0x46d4); // move to TSNUM1 offset
    *param_ptr++ = 0x00000000; // TSNUM1
    *param_ptr++ = 0x80000000; // TSNUM2
    *param_ptr++ = 0x00000000; // TSNUM3
    *param_ptr++ = 0x00000000; // TSNUM4

    //BPRMTR 
    param_ptr = (unsigned int *)(IMMRBAR + QE_BASE + 0x40a0);
    *param_ptr = 0x00000000; // BPRMTR

    //TCRH 
    param_ptr = (unsigned int *)(IMMRBAR + QE_BASE + 0x4680); // TRB2
    *param_ptr = 0xcc002000; // TCRH
}


void reset_counters_r1(void)
{

unsigned int *tcrh;

    tcrh = (unsigned int *)(IMMRBAR + QE_BASE + 0x4600); // TRB1
    *tcrh   = *tcrh & 0xFFFF3FFF;
    *tcrh   = *tcrh | 0x00004000;
    *tcrh   = *tcrh & 0xFFFF3FFF;
    *tcrh   = *tcrh | 0x00008000;

}


void reset_counters_r2(void)
{

unsigned int *tcrh;

    tcrh = (unsigned int *)(IMMRBAR + QE_BASE + 0x4680); // TRB2
    *tcrh   = *tcrh & 0xFFFF3FFF;
    *tcrh   = *tcrh | 0x00004000;
    *tcrh   = *tcrh & 0xFFFF3FFF;
    *tcrh   = *tcrh | 0x00008000;

}


void dump_trb_r1(int dump_index)
{

#define TESR_RAM_OFFSET 0x000001FF
#define TESR_STOP_BERR  0x05000000
#define TESR_WRAP_COUNT 0x00FF0000

unsigned int *tar,*tdr,*tesr,d1,d2,ram_ptr,stop,ptr,wrap_cnt;
unsigned int *tcrh,*rev_num;

unsigned int i,line_count,start_addr,delay;


    PRINT( "//-------- START trb1_dump_%3d ------------------------ \n", dump_index);
    // file header
    PRINT( "// choose mode from these options: normal,compressed,high_compressed,time_stamp\n");
    PRINT( "// write true or false near time-stamp enabled label\n");
    PRINT( "//\n");

    // get TESR
    tesr = (unsigned int *)(IMMRBAR + QE_BASE + 0x4608);
    // disable counters in TCRH (bit 0x00004000)
    tcrh = (unsigned int *)(IMMRBAR + QE_BASE + 0x4600);
    *tcrh = *tcrh | 0x00004000;
    // disable TCRH (bit 0x80000000)
    *tcrh = *tcrh & 0x7FFFFFFF;

    // print microcode REV_NUM register to enable us to identify the correct ROM MAP file
    rev_num = (unsigned int *)(IMMRBAR + QE_BASE + 0x1B8);
    PRINT( "//\n");
    PRINT( "// REV_NUM: 0x%08X\n",*rev_num);

    // print TESR
    PRINT( "//\n");
    PRINT( "// TESR: 0x%08X\n",*tesr);

    PRINT( "\n");
    PRINT( "\n");
    // print dump settings
    PRINT( "mode: normal\n");
    PRINT( "time-stamp enabled: false\n");
    PRINT( "\n");

    // trace dump

    tar    = (unsigned int *)(IMMRBAR + QE_BASE + 0x464c);
    tdr    = (unsigned int *)(IMMRBAR + QE_BASE + 0x4650);

    stop    = *tesr && TESR_STOP_BERR;
    ram_ptr = *tesr & TESR_RAM_OFFSET;
    wrap_cnt= (*tesr & TESR_WRAP_COUNT) >> 16;

    line_count = 0;
    start_addr = 0;

    if (wrap_cnt > 0)
    {
        line_count = 400;
        start_addr = ram_ptr + 1;
    }
    else
    {
        line_count = ram_ptr + 1;
    }


    for (i=start_addr; i<(start_addr + line_count) ;i++)
    {
        ptr = ((i*8) % 3200) ;
        *tar = ptr;
		asm volatile("msync; isync");
        delay++; // Use mbar/eieio Power Architecture instruction here to ensure correct st/ld ordering!
        d1 = *tdr;

        ptr = ((i*8) % 3200)+4;
        *tar = ptr;
		asm volatile("msync; isync");
        delay++; // Use mbar/eieio Power Architecture instruction here to ensure correct st/ld ordering!
        d2 = *tdr;

        PRINT( "0x%08X%08X\n",d1, d2);
    }

    // enable counters in TCRH (bit 0x00004000)
    *tcrh = *tcrh & 0xFFFFBFFF;
    // reset TESR
    *tesr = 0xffff7fff;
    // enable TCRH (bit 0x80000000)
    *tcrh = *tcrh | 0x80000000;


    PRINT( "//-------- END trb1_dump_%3d ------------------------ \n", dump_index);
    PRINT( "\n");

}


void dump_trb_r2(int dump_index)
{

#define TESR_RAM_OFFSET 0x000001FF
#define TESR_STOP_BERR  0x05000000
#define TESR_WRAP_COUNT 0x00FF0000

unsigned int *tar,*tdr,*tesr,d1,d2,ram_ptr,stop,ptr,wrap_cnt;
unsigned int *tcrh,*rev_num;

unsigned int i,line_count,start_addr,delay;


    PRINT( "//-------- START trb2_dump_%3d ------------------------ \n", dump_index);
    // file header
    PRINT( "// choose mode from these options: normal,compressed,high_compressed,time_stamp\n");
    PRINT( "// write true or false near time-stamp enabled label\n");
    PRINT( "//\n");

    // get TESR
    tesr = (unsigned int *)(IMMRBAR + QE_BASE + 0x4688);
    // disable counters in TCRH (bit 0x00004000)
    tcrh = (unsigned int *)(IMMRBAR + QE_BASE + 0x4680);
    *tcrh = *tcrh | 0x00004000;
    // disable TCRH (bit 0x80000000)
    *tcrh = *tcrh & 0x7FFFFFFF;

    // print microcode REV_NUM register to enable us to identify the correct ROM MAP file
    rev_num = (unsigned int *)(IMMRBAR + QE_BASE + 0x1B8);
    PRINT( "//\n");
    PRINT( "// REV_NUM: 0x%08X\n",*rev_num);

    // print TESR
    PRINT( "//\n");
    PRINT( "// TESR: 0x%08X\n",*tesr);

    PRINT( "\n");
    PRINT( "\n");
    // print dump settings
    PRINT( "mode: normal\n");
    PRINT( "time-stamp enabled: false\n");
    PRINT( "\n");

    // trace dump

    tar    = (unsigned int *)(IMMRBAR + QE_BASE + 0x46cc);
    tdr    = (unsigned int *)(IMMRBAR + QE_BASE + 0x46d0);

    stop    = *tesr && TESR_STOP_BERR;
    ram_ptr = *tesr & TESR_RAM_OFFSET;
    wrap_cnt= (*tesr & TESR_WRAP_COUNT) >> 16;

    line_count = 0;
    start_addr = 0;

    if (wrap_cnt > 0)
    {
        line_count = 400;
        start_addr = ram_ptr + 1;
    }
    else
    {
        line_count = ram_ptr + 1;
    }


    for (i=start_addr; i<(start_addr + line_count) ;i++)
    {
        ptr = ((i*8) % 3200) ;
        *tar = ptr;
		asm volatile("msync; isync");
        delay++; // Use mbar/eieio Power Architecture instruction here to ensure correct st/ld ordering!
        d1 = *tdr;

        ptr = ((i*8) % 3200)+4;
        *tar = ptr;
		asm volatile("msync; isync");
        delay++; // Use mbar/eieio Power Architecture instruction here to ensure correct st/ld ordering!
        d2 = *tdr;

        PRINT( "0x%08X%08X\n",d1,d2);
    }

    // enable counters in TCRH (bit 0x00004000)
    *tcrh = *tcrh & 0xFFFFBFFF;
    // reset TESR
    *tesr = 0xffff7fff;
    // enable TCRH (bit 0x80000000)
    *tcrh = *tcrh | 0x80000000;


    PRINT( "//-------- END trb2_dump_%3d ------------------------ \n", dump_index);
    PRINT( "\n");

}


void dump_pm_r1(int dump_index)
{

unsigned int *tcrh;
unsigned int *tpcch,*tpccl,*tpc1h,*tpc1l,*tpc2h,*tpc2l;
//unsigned long long main_count,count1,count2,lose,eff;
u64 main_count,count1,count2,lose,eff;


    tcrh = (unsigned int *)(IMMRBAR + QE_BASE + 0x4600);
    *tcrh = *tcrh | 0x00004000;

    // get counters
    if (QE_REV == 1)
    {
        tpcch    = (unsigned int *)(IMMRBAR + QE_BASE + 0x4630);
        tpccl    = (unsigned int *)(IMMRBAR + QE_BASE + 0x462c);
    }
    else // QE_REV = 2
    {
        tpcch    = (unsigned int *)(IMMRBAR + QE_BASE + 0x462c);
        tpccl    = (unsigned int *)(IMMRBAR + QE_BASE + 0x4630);
    }
    tpc1h    = (unsigned int *)(IMMRBAR + QE_BASE + 0x4634);
    tpc1l    = (unsigned int *)(IMMRBAR + QE_BASE + 0x4638);
    tpc2h    = (unsigned int *)(IMMRBAR + QE_BASE + 0x463c);
    tpc2l    = (unsigned int *)(IMMRBAR + QE_BASE + 0x4640);

    // caculate LDSCH LOSE and CODE EFFICIENCY (if PM was not set otherwise)
    if ( 1 )
    {
        main_count = (((unsigned long long)*tpcch << 32)) + *tpccl;
        count1 = (((unsigned long long)*tpc1h << 32)) + *tpc1l;
        count2 = (((unsigned long long)*tpc2h << 32)) + *tpc2l;
		//lose = ((count1*100) / main_count);
        lose = count1*100;
        do_div(lose, main_count);

        if (main_count != count1)
		{
			//eff = 100 - (count2*100/(main_count - count1));
            eff = count2*100;
			do_div(eff, (main_count - count1));
            eff = 100 - eff;
		}
        else
            eff = 0;
    }

    PRINT( "//-------- START pm1_dump_%3d ------------------------ \n", dump_index);
    // print counters
    PRINT( "// main count: 0x%08X%08X\n",*tpcch,*tpccl);
    PRINT( "// 1st  count: 0x%08X%08X\n",*tpc1h,*tpc1l);
    PRINT( "// 2nd  count: 0x%08X%08X\n",*tpc2h,*tpc2l);
    PRINT( "//\n");
    // maybe print calculations
    if ( 1 )
    {
        if (main_count)
            PRINT( "// RISC 1 LDSCH LOSE:%llu%% LDSCH:%llu CYCLES:%llu\n",lose, count1, main_count);
        if (eff)
            PRINT( "// RISC 1 CODE EFFICIENCY:%llu%% OTHER STALLS:%llu CYCLES:%llu\n",eff, count2, main_count);
    }

    // enable counters in TCRH (bit 0x00004000)
    *tcrh = *tcrh & 0xFFFFBFFF;

    PRINT( "//\n");
    PRINT( "//-------- END pm1_dump_%3d ------------------------ \n", dump_index);
    PRINT( "\n");

}


void dump_pm_r2(int dump_index)
{

unsigned int *tcrh;
unsigned int *tpcch,*tpccl,*tpc1h,*tpc1l,*tpc2h,*tpc2l;
unsigned long long main_count,count1,count2,lose,eff;


    tcrh = (unsigned int *)(IMMRBAR + QE_BASE + 0x4680);
    *tcrh = *tcrh | 0x00004000;

    // get counters
    if (QE_REV == 1)
    {
        tpcch    = (unsigned int *)(IMMRBAR + QE_BASE + 0x46b0);
        tpccl    = (unsigned int *)(IMMRBAR + QE_BASE + 0x46ac);
    }
    else // QE_REV = 2
    {
        tpcch    = (unsigned int *)(IMMRBAR + QE_BASE + 0x46ac);
        tpccl    = (unsigned int *)(IMMRBAR + QE_BASE + 0x46b0);
    }
    tpc1h    = (unsigned int *)(IMMRBAR + QE_BASE + 0x46b4);
    tpc1l    = (unsigned int *)(IMMRBAR + QE_BASE + 0x46b8);
    tpc2h    = (unsigned int *)(IMMRBAR + QE_BASE + 0x46bc);
    tpc2l    = (unsigned int *)(IMMRBAR + QE_BASE + 0x46c0);

    // caculate LDSCH LOSE and CODE EFFICIENCY (if PM was not set otherwise)
    if ( 1 )
    {
        main_count = (((unsigned long long)*tpcch << 32)) + *tpccl;
        count1 = (((unsigned long long)*tpc1h << 32)) + *tpc1l;
        count2 = (((unsigned long long)*tpc2h << 32)) + *tpc2l;
		// lose = ((count1*100) / main_count);
        lose = count1*100;
        do_div(lose , main_count);

        if (main_count != count1)
		{
			// eff = 100 - (count2*100/(main_count - count1));
			eff = count2*100;
			do_div(eff, (main_count - count1));
            eff = 100 - eff;
		}
        else
            eff = 0;
    }

    PRINT( "//-------- START pm2_dump_%3d ------------------------ \n", dump_index);
    // print counters
    PRINT( "// main count: 0x%08X%08X\n",*tpcch,*tpccl);
    PRINT( "// 1st  count: 0x%08X%08X\n",*tpc1h,*tpc1l);
    PRINT( "// 2nd  count: 0x%08X%08X\n",*tpc2h,*tpc2l);
    PRINT( "//\n");
    // maybe print calculations
    if ( 1 )
    {
        if (main_count)
            PRINT( "// RISC 1 LDSCH LOSE:%d%% LDSCH:%llu CYCLES:%llu\n",lose, count1, main_count);
        if (eff)
            PRINT( "// RISC 1 CODE EFFICIENCY:%d%% OTHER STALLS:%llu CYCLES:%llu\n",eff, count2, main_count);
    }

    // enable counters in TCRH (bit 0x00004000)
    *tcrh = *tcrh & 0xFFFFBFFF;

    PRINT( "//\n");
    PRINT( "//-------- END pm2_dump_%3d ------------------------ \n", dump_index);
    PRINT( "\n");

}


