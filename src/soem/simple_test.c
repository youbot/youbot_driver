/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

char IOmap[4096];

void simpletest(char *ifname)
{
	int i, j, oloop, iloop, wkc, wkc_count, slave;
	boolean needlf;

	needlf = FALSE;
	printf("Starting simple test\n");
	
	/* initialise SOEM, bind socket to ifname */
	if (ec_init(ifname))
	{	
		printf("ec_init on %s succeeded.\n",ifname);
		/* find and auto-config slaves */


	    if ( ec_config_init(FALSE) > 0 )
		{
			printf("%d slaves found and configured.\n",ec_slavecount);

			ec_config_map(&IOmap);

			printf("Slaves mapped, state to SAFE_OP.\n");
			/* wait for all slaves to reach SAFE_OP state */
			ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

			oloop = ec_slave[0].Obytes;
			if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
			if (oloop > 8) oloop = 8;
			iloop = ec_slave[0].Ibytes;
			if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
			if (iloop > 8) iloop = 8;

			printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

			printf("Request operational state for all slaves\n");
			printf("Calculated workcounter %d\n",ec_group[0].expectedWKC);
			ec_slave[0].state = EC_STATE_OPERATIONAL;
			/* send one valid process data to make outputs in slaves happy*/
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);
			/* request OP state for all slaves */
			ec_writestate(0);
			/* wait for all slaves to reach OP state */
			ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE * 4);
			if (ec_slave[0].state == EC_STATE_OPERATIONAL )
			{
				printf("Operational state reached for all slaves.\n");
				wkc_count = 0;
				/* cyclic loop 10 times */
				for(i = 1; i <= 10000; i++)
				{
					ec_send_processdata();
					wkc = ec_receive_processdata(EC_TIMEOUTRET);

					if( (wkc < ec_group[0].expectedWKC) || ec_group[0].docheckstate)
					{
						if (needlf)
						{
							needlf = FALSE;
							printf("\n");
						}
						/* one ore more slaves are not responding */
						ec_group[0].docheckstate = FALSE;
						ec_readstate();
						for (slave = 1; slave <= ec_slavecount; slave++)
						{
							if (ec_slave[slave].state != EC_STATE_OPERATIONAL)
							{
								ec_group[0].docheckstate = TRUE;
								if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
								{
									printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
									ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
									ec_writestate(slave);
								}
								else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
								{
									printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
									ec_slave[slave].state = EC_STATE_OPERATIONAL;
									ec_writestate(slave);										
								}
								else if(ec_slave[slave].state > 0)
								{
									if (ec_reconfig_slave(slave))
									{
										ec_slave[slave].islost = FALSE;
										printf("MESSAGE : slave %d reconfigured\n",slave);									
									}
								} 
								else if(!ec_slave[slave].islost)
								{
									ec_slave[slave].islost = TRUE;
									printf("ERROR : slave %d lost\n",slave);									
								}
							}
							if (ec_slave[slave].islost)
							{
								if(!ec_slave[slave].state)
								{
									if (ec_recover_slave(slave))
									{
										ec_slave[slave].islost = FALSE;
										printf("MESSAGE : slave %d recovered\n",slave);									
									}
								}
								else
								{
									ec_slave[slave].islost = FALSE;
									printf("MESSAGE : slave %d found\n",slave);									
								}
							}
						}
						if(!ec_group[0].docheckstate)
							printf("OK : all slaves resumed OPERATIONAL.\n");
					}
					else
					{
						printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

						for(j = 0 ; j < oloop; j++)
						{
							printf(" %2.2x", *(ec_slave[0].outputs + j));
						}

						printf(" I:");					
						for(j = 0 ; j < iloop; j++)
						{
							printf(" %2.2x", *(ec_slave[0].inputs + j));
						}	
						printf("\r");
						needlf = TRUE;
					}
					usleep(10000);
					
				}
			}
			else
			{
				printf("Not all slaves reached operational state.\n");
				ec_readstate();
				for(i = 1; i<=ec_slavecount ; i++)
				{
					if(ec_slave[i].state != EC_STATE_OPERATIONAL)
					{
						printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
							i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
					}
				}
			}			
			printf("\nRequest safe operational state for all slaves\n");
			ec_slave[0].state = EC_STATE_INIT;
			/* request SAFE_OP state for all slaves */
			ec_writestate(0);
		}
		else
		{
			printf("No slaves found!\n");
		}
		printf("End simple test, close socket\n");
		/* stop SOEM, close socket */
		ec_close();
	}
	else
	{
		printf("No socket connection on %s\nExcecute as root\n",ifname);
	}	
}	

int main(int argc, char *argv[])
{
	printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

	if (argc > 1)
	{		
		/* start cyclic part */
		simpletest(argv[1]);
	}
	else
	{
		printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
	}	
	
	printf("End program\n");
	return (0);
}
