#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <termios.h>
#include <sys/file.h>

static uint16_t crc(const unsigned char *p, int n) {
    uint16_t c=0;
    while(n--) { c ^= (uint16_t)*p++ << 8; for(int i=0;i<8;i++) c=(c&0x8000)?(c<<1)^0x1021:c<<1; }
    return c;
}
int main(int argc,char **argv) {
    if(argc!=3 || (strcmp(argv[2],"status") && strcmp(argv[2],"stop") && strcmp(argv[2],"start"))) return 2;
    int lock=open("/tmp/rov-gimbal-motor.lock",O_CREAT|O_RDWR,0600);
    if(lock<0 || flock(lock,LOCK_EX|LOCK_NB)) return 8;
    int command=!strcmp(argv[2],"start")?2:3;
    int pid=atoi(argv[1]); if(pid<2) return 2;
    char path[80],exe[256]={0};
    snprintf(path,sizeof path,"/proc/%d/exe",pid);
    if(readlink(path,exe,sizeof exe-1)<0 || strcmp(exe,"/opt/bin/gcu/gb_control")) return 3;
    snprintf(path,sizeof path,"/proc/%d/mem",pid);
    int mem=open(path,O_RDONLY); if(mem<0){perror("memory read");return 4;}
    /* These read-only snapshot addresses are for the inspected executable only. */
    unsigned char tx[40],rx[26];
    int valid=0;
    for(int i=0;i<1000;i++) {
        if(pread(mem,rx,26,0x8e418)==26 && rx[0]==0xb5 && rx[1]==0x9a && !crc(rx,26)){valid=1;break;}
        usleep(1000);
    }
    if(!valid){fprintf(stderr,"No valid cached reply: ");for(int i=0;i<26;i++)fprintf(stderr,"%02x ",rx[i]);fprintf(stderr,"\n");close(mem);return 7;}
    int guard=fork();
    if(guard<0) return 5;
    if(!guard){sleep(2);kill(pid,SIGCONT);_exit(0);}
    int result=6,fd=-1;
    if(kill(pid,SIGSTOP)) goto done;
    usleep(30000);
    if(pread(mem,tx,40,0x8e398)!=40) goto done;
    if(tx[0]!=0xa9 || tx[1]!=0x5b || crc(tx,40) || rx[0]!=0xb5 || rx[1]!=0x9a || crc(rx,26)) {
        fprintf(stderr,"Snapshot invalid; nothing sent\n");goto done;
    }
    printf("Before: firmware=%u hardware_error=%u state=%u command=%u result=%u\n",rx[2],rx[3],(rx[4]>>1)&7,rx[5]>>3,rx[5]&7);
    if(!strcmp(argv[2],"status")){result=0;goto done;}
    fd=open("/dev/ttyS3",O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(fd<0) goto done;
    /* Preserve all existing pose/auxiliary fields; alter command and CRC only. */
    tx[2]=(command<<3)|((tx[2]+1)&7);
    uint16_t c=crc(tx,38);tx[38]=c>>8;tx[39]=c;
    tcflush(fd,TCIFLUSH);
    if(write(fd,tx,40)!=40) goto done;
    unsigned char buffer[512];int used=0;
    for(int t=0;t<50;t++) {
        usleep(10000);
        int n=read(fd,buffer+used,sizeof buffer-used);
        if(n>0) used+=n;
        for(int i=0;i+26<=used;i++) if(buffer[i]==0xb5 && buffer[i+1]==0x9a && !crc(buffer+i,26)) {
            unsigned char *r=buffer+i;
            printf("Reply: firmware=%u hardware_error=%u state=%u command=%u result=%u\n",r[2],r[3],(r[4]>>1)&7,r[5]>>3,r[5]&7);
            int state=(r[4]>>1)&7;
            if((r[5]>>3)==command && (r[5]&7)==1 &&
               (command==3 ? state==2 : state==4)) {
                tx[2]&=7;c=crc(tx,38);tx[38]=c>>8;tx[39]=c;
                if(write(fd,tx,40)!=40) goto done;
                usleep(10000); result=0;goto done;
            }
        }
        if(used>450) used=0;
    }
    fprintf(stderr,"Motor command not confirmed; no further commands sent\n");
done:
    if(fd>=0) close(fd);
    close(mem);
    kill(pid,SIGCONT);
    kill(guard,SIGTERM);waitpid(guard,NULL,0);
    printf("Camera controller resumed; result=%d\n",result);
    return result;
}
