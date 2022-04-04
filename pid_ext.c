#include <stdio.h>
#include "dlab_def.h"
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <math.h>

#define MAXS 6000
#define PI M_PI
float Kp, run_time, Fs, theta[MAXS], ref[MAXS], magnitude, frequency, duty_cycle, sqfreq;
float Ti, Td, N, Tt;
int motor_num, no_of_samples;
char selection;
sem_t data_avail;

float satblk(float input){
    float ut;
    if(input>=-3 && input<=3){
        ut = input;
    }
    else if(input>3){
        ut = 3.0;
    }
    else if(input<-3){
        ut = -3.0;
    }
    return ut;
}       

void *Control(void *arg){
    int k=0;
    no_of_samples=(int)(run_time*Fs);
    float motor_position;
    float ek = 0;   //tracking error
    float uk = 0;  //calculate control value
    float ak = 0,ak_pre = 0;
    float ek_pre = 0, ik = 0, ik_pre = 0, dk = 0, dk_pre = 0, pid = 0, pid_anti = 0;
    float Ts = 1/Fs;

    while(k<no_of_samples){
        sem_wait(&data_avail);
        motor_position=EtoR(ReadEncoder());
        ek=ref[k]-motor_position;
        uk=Kp*ek;
        ik=ik_pre+(Kp/Ti)*(ek_pre)*Ts+(1/Tt)*ak_pre*Ts;
        dk = (Td/(N*Ts+Td))*dk_pre + ((Kp*Td*N)/(N*Ts+Td))*(ek-ek_pre);
        pid = uk + ik + dk;
        pid_anti = satblk(pid);
        if(pid>=-3 && pid<=3){  
          ak = 0;
        }
        else if(pid>3){
          ak = pid_anti - pid;
        }
        else if(pid<-3){
          ak = -1*pid - (-1*pid_anti);
        }
    
        DtoA(VtoD(pid_anti));
        theta[k]=motor_position;
        ek_pre = ek;
        ik_pre = ik;
        dk_pre = dk;
        ak_pre = ak;
        k++;

        //sem_post(&data_avail);
	//pthread_mutex_unlock(&completion);
	if(k>no_of_samples){
	  k = 0;
    	}
    }
    pthread_exit(NULL);
}

int main()
{
    pthread_t control;
    char user_input[8];
    Kp=58.2; //initializing Kp to 58.2, since this equals to 0.6*Ku
    run_time=3.0; 
    Fs=200.0; //initial frequecy is 200Hz
    motor_num=1; //motor number is 1
    magnitude=5*M_PI/18; // default angle given in lab manual (50 deg)
    Ti = 0.035; // 0.5*Pu
    Td = 0.00875; // 1/8 of Pu
    Tt = 0.01;
    N = 20; 
    sqfreq = 0.5;
    duty_cycle = 50;
    

// Initialize the reference input vector ref[k].   
    no_of_samples=(int)(run_time*Fs);
    int i;
    for(i=0; i<no_of_samples;i++){
        ref[i]=magnitude;               
    }
// Initialize the reference input vector ref[k].   

    while(1){
       

        printf("\t Feel free to enter a command\n");
        printf("r: Run the control algorithm \n");
        printf("p: Change value of Kp \n");
        printf("f: Change value of sample frequency, Fs \n");
        printf("t: Change value of run_time, Tf \n");
        printf("u: Change the type of inputs(Step or Square) \n");
        printf("g: Plot results on screen \n");
        printf("i: Change the value of Ti \n");
        printf("d: Change the value of Td \n");
        printf("n: Change the value of N \n");
        printf("h: Save the plot results in Postscript \n");
        printf("q: exit \n \n"); 
        scanf(" %c",&selection);
        switch(selection){
        case 'r':   
            printf("\n Case r \n");
            no_of_samples=(int)(run_time*Fs);
            int i;
            // Initialize the reference input vector ref[k].   
            if(strcmp(user_input,"step")==0){
               for(i=0; i<no_of_samples;i++){
                 ref[i]=magnitude;               
               } 
            }
            if(strcmp(user_input,"square")==0){
                Square(ref,no_of_samples,Fs,magnitude,sqfreq,duty_cycle);
            }
            // Initialize the reference input vector ref[k].   
            sem_init(&data_avail,0,1);
            printf("\n Initializing Motor \n");
            Initialize(Fs,motor_num); 
            printf("\n Creating Thread \n \n");
            pthread_create(&control,NULL,&Control,NULL);
            pthread_join(control,NULL);
            Terminate();
            sem_destroy(&data_avail);
            break;
        case 'u':
            printf("\n Case U \n");   
            printf(" Choose Step or Square input \n \n"); 
            scanf("%s",user_input);
            if(strcmp(user_input,"step")==0){
				printf("Enter the Magnitude of Step\n");
                scanf("%f", &magnitude);
                magnitude=magnitude*M_PI/180;//deg to rad 
                printf("New Step Magnitude = %f \n", magnitude);
            } 
            if(strcmp(user_input,"square")==0){
				printf("Enter the Magnitude of Square\n");
                scanf("%f", &magnitude);
                magnitude=magnitude*M_PI/180;//deg to rad 
                printf("New Square Magnitude = %f \n", magnitude);
                printf("Enter the frequency of Square\n");
                scanf("%f", &sqfreq);
                printf("New Square frequency = %f \n", sqfreq);
                printf("Enter the duty cycle of Square\n");
                scanf("%f", &duty_cycle);
                printf("New Square duty cycle = %f \n", duty_cycle);
                
            } 
           
            break;
        case 'p':
            printf("\n Case P \n");   
            printf("Enter new value for Kp \n"); 
            scanf("%f",&Kp); 
            printf("New Kp value = %f \n", Kp);
            
            break;
        case 'g':
            printf("\n Case G: stop \n");
            plot(ref,theta,Fs,no_of_samples,SCREEN,"Proportional Control (Kp)","Time(sec)","Rotation(rad)");
            
            break;
        case 'i': 
            printf("\n Case I \n");
            printf(" Enter a new Ti:\n");
            scanf("%f",&Ti);
            printf("New Ti value = %f \n", Ti);
	    break;
		   
        case 'd': 
            printf("\n Case D \n");
            printf(" Enter a new Td:\n");
            scanf("%f",&Td);
            printf("New Td value = %f \n", Td);
	    break;
		   
        case 'n': 
            printf("\n Case N \n");
            printf(" Enter a new N :\n");
            scanf("%f",&N);
            printf("New N value = %f \n", N);
	    break;
		 
        case 'h':
            printf("\n Case H \n");
            plot(ref,theta,Fs,no_of_samples,PS,"Proportional Control (Kp)","Time(sec)","Rotation(rad)");
           
            break;
        case 'q': 
            printf("\n Case Q \n");
            printf("Done! \n");
            
            exit(0);
	    break;
        case 'f': 
            printf("\n Case F \n");
            printf(" Enter a new Frequency :\n");
            scanf("%f",&Fs);
            printf("New Fs value = %f \n", Fs);
		  
            break;
        case 't':
            printf("\n Case T \n");
            printf("Enter Run Time :\n");
            scanf("%f",&run_time);
            printf("New Tf value = %f \n", run_time);
           
            break;
   
        default:
            printf("\n Case DEFAULT \n");
            printf("Invalid\n");
		 
            break;
        }
       
    }
    return 0;
}
