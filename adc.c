#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/fs.h>
#include <errno.h>
#include <string.h>
#include <math.h>

#define PWM_IOCTL_SET_FREQ		1
#define PWM_IOCTL_STOP			0
#define Ramp_up_constant		10
#define Ramp_down_constant		10
#define deltaT				1000

#define	ESC_KEY		0x1b

//FFT calculation for obtaining power spectrum section
struct Complex
{	double a;        //for storing Real Part
	double b;        //for storing Imaginary Part
}X[257], U, W, T, Tmp;

void FFT(void)
{
	int M=7,i=1,j=1,k=1,LE=0,LE1=0,IP=0;
	int N=pow(2.0,M);
	for(k=1;k<=M;k++)
	{
		LE=pow(2.0,M+1-k);
		LE1 = LE / 2;
		U.a = 1.0;
		U.b = 0.0;
		W.a = cos(M_PI / (double)LE1);
		W.b = -sin(M_PI / (double)LE1);
		for (j = 1; j <= LE1; j++)
		{
			for (i = j; i <= N; i = i + LE)
			{
				IP = i + LE1;
				T.a = X[i].a + X[IP].a;
				T.b = X[i].b + X[IP].b;
				Tmp.a = X[i].a - X[IP].a;				
				Tmp.b = X[i].b - X[IP].b;
				X[IP].a = (Tmp.a * U.a) - (Tmp.b * U.b);
				X[IP].b = (Tmp.a * U.b) + (Tmp.b * U.a);
				X[i].a = T.a;
				X[i].b = T.b;
			}
			Tmp.a = (U.a * W.a) - (U.b * W.b);
			Tmp.b = (U.a * W.b) + (U.b * W.a);
			U.a = Tmp.a;
			U.b = Tmp.b;
		}
	}
	int NV2 = N/2;
	int NM1 = N-1;
	int K = 0;
	j = 1;
	for (i = 1; i <= NM1; i++)
	{
		if (i >= j) goto TAG25;
		T.a = X[j].a;
		T.b = X[j].b;
		X[j].a = X[i].a;
		X[j].b = X[i].b;
		X[i].a = T.a;
		X[i].b = T.b;
TAG25:	K = NV2;
TAG26:	if (K >= j) goto TAG30;
		j = j - K;
		K = K / 2;
		goto TAG26;
TAG30:	j = j + K;
	}
}
//FFT complete

static int getch(void)
{
	struct termios oldt,newt;
	int ch;

	if (!isatty(STDIN_FILENO)) {
		fprintf(stderr, "this problem should be run at a terminal\n");
		exit(1);
	}
	// save terminal setting
	if(tcgetattr(STDIN_FILENO, &oldt) < 0) {
		perror("save the terminal setting");
		exit(1);
	}

	// set terminal as need
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	if(tcsetattr(STDIN_FILENO,TCSANOW, &newt) < 0) {
		perror("set terminal");
		exit(1);
	}

	ch = getchar();

	// restore termial setting
	if(tcsetattr(STDIN_FILENO,TCSANOW,&oldt) < 0) {
		perror("restore the termial setting");
		exit(1);
	}
	return ch;
}

static int fd = -1;
static void close_buzzer(void);
static void open_buzzer(void)
{
	fd = open("/dev/himanshu_pwm", 0);
	if (fd < 0) {
		perror("open pwm_buzzer device");
		exit(1);
	}

	// any function exit call will stop the buzzer
	atexit(close_buzzer);
}

static void close_buzzer(void)
{
	if (fd >= 0) {
		ioctl(fd, PWM_IOCTL_STOP);
		if (ioctl(fd, 2) < 0) {
			perror("ioctl 2:");
		}
		close(fd);
		fd = -1;
	}
}

static void set_buzzer_freq(int freq,int duty)
{
	// this IOCTL command is the key to set frequency
	int ret = ioctl(fd, freq,duty);  //int ret = ioctl(fd, PWM_IOCTL_SET_FREQ, freq,duty);
	if(ret < 0) {
		perror("set the frequency of the buzzer");
		exit(1);
	}
}
static void stop_buzzer(void)
{
	int ret = ioctl(fd, 0);
	if(ret < 0) {
		perror("stop the buzzer");
		exit(1);
	}
	if (ioctl(fd, 2) < 0) {
		perror("ioctl 2:");
	}
}

static void soft_start(void)
{
	int freq_lo=0;
	int duty=50;
	int i; 
	for (i=1000; i>0;i-10)
{
	freq_lo=i;
	set_buzzer_freq(freq_lo,duty);
	usleep(1000);
}
}

int main(int argc, char **argv)
{		
	int freq = 800;
	int fd_l;
	int duty;
	int on;
	open_buzzer();
	sscanf(argv[1],"%d", &duty);  // enter the duty cycle from the terminal
	sscanf(argv[2],"%d", &on);    // give motor direction from the termial	

	fd_l = open("/dev/himanshu_gpio", 0);
	if (fd_l < 0) {
		fd = open("/dev/himanshu_gpio", 0);
	}
	if (fd_l < 0) {
		perror("open device leds");
		exit(1);
	}

	ioctl(fd_l, on);
	close(fd_l);

	printf( "\nBUZZER TEST ( PWM Control )\n" );
	printf( "Press +/- to increase/reduce the frequency of the BUZZER\n" ) ;
	printf( "Press 'ESC' key to Exit this program\n\n" );
	////////////ADC
		//for DFT	
	int i=0;
	int arr[300];
	float power[256];
	float mean;
		//dft over
	int fd_a ;
	fd_a= open("/dev/adc_lab", 0);
	if (fd_a < 0) {
		perror("open ADC device:");
		return 1;
	}
	
	// for setting up frequency for rotation
	
	for(i=1;i<=256;i++) {
		char buffer[30];
		int len = read(fd_a, buffer, sizeof buffer -1);
		if (len > 0) {
			buffer[len] = '\0';
			int value = -1;
			sscanf(buffer, "%d", &value);
			//Mapping adc value to frequency
				if(value<5){
				soft_start();
				stop_buzzer();
				}

				freq= value + 1000;
		
				set_buzzer_freq(freq,duty);						
			printf("ADC Value: %d\n", value);
			arr[i]=value;
			//getchar();
		} else {
			perror("read ADC device:");
			return 1;
		}
		usleep(500* 1000);
		printf( "\tFreq = %d\n", freq );
		printf( "\tduty = %d\n", duty );
	}
	
	for (i = 1; i <= 256; i++)
	{
		X[i].a = arr[i];
		X[i].b = 0.0;
	}
	printf ("*********Before*********\n");
	for (i = 1; i <= 256; i++)
		printf ("X[%d]:real == %f  imaginary == %f\n", i, X[i].a, X[i].b);
	
	FFT();
	
	for (i = 1; i <= 256; i++)
	{
		X[i].a = X[i].a/256;
		X[i].b = X[i].b/256;
	}
	printf ("\n\n**********After*********\n");
	for (i = 1; i <= 256; i++)
		printf ("X[%d]:real == %f  imaginary == %f\n", i, X[i].a, X[i].b);
	//char buffer[1000];	
	for(i=1;i<=256; i++)
	{
		power[i]= sqrt(((X[i].a*X[i].a)+(X[i].b*X[i].b)));
		printf("power spectrum value of power[%d] is = %f \n",i,power[i]);
		//sprintf(buffer, "%f",power[i]);
		fprintf(fp,"%f,\n",power[i]);
	}
	mean=0;
	for(i=102;i<=154;i++)
	{
		mean= mean+ power[i];
	}
	mean/=52;
	printf("mean is %f \n",mean);
	if(mean<(0.15*(power[127])))
		printf("values are valid \n");
	else
printf("values are not valid \n");

	close(fd_a);
return 0;
}
