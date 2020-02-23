#include <linux/init.h> //
#include <linux/interrupt.h> //
#include <linux/io.h> //
#include <linux/kernel.h> //
#include <linux/module.h> //
#include <linux/of_address.h>
#include <linux/of_device.h> //
#include <linux/of_irq.h> /* "Open Firmware" */
#include <linux/platform_device.h>

#include <linux/delay.h>

#define DRIVER_NAME "bcm2835timer"
#define LOCAL_CONTROL	0x000
#define LOCAL_PRESCALER	0x008
#define LOCAL_TIMER_CONTROL 0x034

#define REG_CONTROL     0x00
#define REG_COUNTER_LO  0x04
#define REG_COUNTER_HI  0x08
#define REG_COMPARE(n)  (0x0c + (n) * 4)
#define MAX_TIMER       3
#define DEFAULT_TIMER   1
// #define INTER 9830 // 4096  50 Hz 
// #define INTER 4096 
// #define INTER 4088 
#define INTER 4080 

/* GPIO register offsets */
#define GPFSEL0         0x0     /* Function Select */
#define GPSET0          0x1c    /* Pin Output Set */
#define GPCLR0          0x28    /* Pin Output Clear */
#define GPLEV0          0x34    /* Pin Level */
#define GPEDS0          0x40    /* Pin Event Detect Status */
#define GPREN0          0x4c    /* Pin Rising Edge Detect Enable */
#define GPFEN0          0x58    /* Pin Falling Edge Detect Enable */
#define GPHEN0          0x64    /* Pin High Detect Enable */
#define GPLEN0          0x70    /* Pin Low Detect Enable */
#define GPAREN0         0x7c    /* Pin Async Rising Edge Detect */
#define GPAFEN0         0x88    /* Pin Async Falling Edge Detect */
#define GPPUD           0x94    /* Pin Pull-up/down Enable */
#define GPPUDCLK0       0x98    /* Pin Pull-up/down Enable Clock */
 
#define FSEL_REG(p)             (GPFSEL0 + (((p) / 10) * 4))
#define FSEL_SHIFT(p)           (((p) % 10) * 3)
#define GPIO_REG_OFFSET(p)      ((p) / 32)
#define GPIO_REG_SHIFT(p)       ((p) % 32)
 
// #define HW_REGISTER_RW(addr) (addr) 

#define BCM2708_PERI_BASE  0x3F000000
#define CM_GP0CTL          (BCM2708_PERI_BASE + 0x101070)

// Macro to set ALT function g=GPIO number, a= Alternate function number 
// Function to be used alone
// g = pin num  a= alternate function (0 =  GPCLK0)
#define SET_GPIO_ALT(pin,alt,gpi) writel(  ((alt)<=3?(alt)+4:(alt)==4?3:2)<<(((pin)%10)*3),  gpi+((pin)/10) )

#define BLOCK_SIZE 4096

#define GSCLK 4
#define BLANK 27
#define XLAT  22
#define VPRG  5
#define CALCUL 17
/*                     How to drive TLC5940
 
   TLC5940 pin definition
      BLANK      = TLC5940 pin 23 = Rpi  gpio 27 
      XLAT       = TLC5940 pin 24 = Rpi  gpio 22
      VPRG       = TLC5940 pin 27 = Rpi  gpio 5
      GSCLK      = TLC5940 pin 18 = Rpi  gpio 4
      CALCUL     = Etat de la pin calcul sur Rpi si 1 si fini et transfer aussii = Rpi gpio 17

   Boucle principale
      Set VPRG to Low

   Sequence d interruption
      1)  set BLANK to High
      2)  Stop 1 MHz oscilator GSCLK after 4096 pulses
      2)  Read "Calcul state" set "Calcul flag" to "yes" or "no"  CALCUL !=0 
      3)  If "Calcul flag" is "yes"
              pulse XLAT
              Set  "Calcul flag" to "no"  clear CALCUL
      4)  Set BLANK to low
      5)  Start 1 MHz oscilator GSCLK for 4096 pulses
*/

// Local functions
static int __init bcm2835timer_probe(struct platform_device *pdev);
static int bcm2835timer_remove(struct platform_device *pdev);
static int init_gpclk (void __iomem *gpio);
static void cleanup_gpclk (void);
static void clk_start(void);
static void clk_stop(void);

static volatile uint32_t CLK_PASSWD,CLK_CTL_MASH,CLK_CTL_SRC,CLK_CTL_ENAB,CLK_DIV_DIVI,CLK_DIV_DIVF,CLK_CTL_KILL;

struct resource res;
struct device_node *np;
void __iomem *clk;

struct resource res;
void __iomem *registers, *gpio_reg;
static unsigned int irq;
volatile unsigned int CountExit, CountEntry;
u32  pinstate,pintemp;
//
static int init_gpclk (void __iomem *gpio)
{
printk ("init gpclk\n");
// 
clk  = ioremap (CM_GP0CTL, 4096);
if (!clk) {
   printk("fail get gpio register \n");
   return -ENOMEM;
}
//
printk("gpio = 0x%08x\n",(unsigned int) gpio);
printk("clk = 0x%08x\n" ,(unsigned int) clk);
//
SET_GPIO_ALT(GSCLK,0,gpio); // Select Alternate function to 0 (GPCLK0) for GPIO 4
//
writel( 0x5a000000, clk);
printk("clk_init= 0x%08x\n", ioread32(clk));
writel( 0x5a000000, clk+ 0x4);
printk("clk_init+1= 0x%08x\n", ioread32(clk+ 0x4));
//
// Definition des parametres de la General Purpose clock O.
CLK_PASSWD  = 0x5A<<24;
printk("CLK_PASSWD = 0x%08x\n", CLK_PASSWD);
//CLK_CTL_MASH= 1 <<9      ;    // To allow fractinal division of clock
CLK_CTL_SRC = 5 <<0      ; // Source is PPLC  1000 MHz
//CLK_CTL_SRC = 6 <<0      ; // Source is PPLD  500 MHz
//CLK_CTL_SRC = 1 <<0      ; // Source is oscillator  19,2 MHz
printk("CLK_CTL_SRC = 0x%08x\n", CLK_CTL_SRC);
CLK_CTL_ENAB= 1<<4    ;
printk("CLK_CTL_ENAB = 0x%08x\n", CLK_CTL_ENAB);
CLK_DIV_DIVI= 1000<<12  ; // Divise source par 1000 donne 1 MHz
//CLK_DIV_DIVI= 4095<<12  ; // Divise source par 1000 donne 1 MHz
//CLK_DIV_DIVI= 19<<12  ; // Divise source par 1000 donne 1 MHz
//CLK_DIV_DIVI= 19<<12  ; // Partie entiere de division source = 19 donne presque 1 MHz
printk("CLK_DIV_DIVI = 0x%08x\n", CLK_DIV_DIVI);
//CLK_DIV_DIVF=   512 <<0 ; // Parie fractionnaire de division 512/1024 =0,2 
CLK_DIV_DIVF=   0 <<0 ; // Parie fractionnaire de division 0 
//
// udelay(100); // Utlise dans tout les programmes
writel( CLK_PASSWD | CLK_DIV_DIVI | CLK_DIV_DIVF, clk+ 0x4);
printk("CM_GP0DIV = 0x%08x\n", (CLK_PASSWD | CLK_DIV_DIVI | CLK_DIV_DIVF));
printk("clk+1= 0x%08x\n", ioread32(clk+ 0x4));
 
writel( CLK_PASSWD | CLK_CTL_MASH | CLK_CTL_SRC | CLK_CTL_ENAB, clk);
printk("CM_GP0CTL = 0x%08x\n", (CLK_PASSWD | CLK_CTL_MASH | CLK_CTL_SRC | CLK_CTL_ENAB));
printk("clk= 0x%08x\n", ioread32(clk));
//
return 0;
}
//
static void init_gpio(unsigned int gpio_pin)
{
//   set gpio_pin as output;
   u32 pintemp;
   pintemp = ioread32(FSEL_REG(gpio_pin)+gpio_reg);
   pintemp &= ~(7 << (((gpio_pin) % 10) * 3));
   pintemp |= (1 << (((gpio_pin) % 10) * 3)); 
   writel(pintemp ,FSEL_REG(gpio_pin)+gpio_reg);          
}
//
static void clk_start(void)
{
   unsigned int clk_read;
   clk_read= ioread32(clk);
//   printk("clk_start= 0x%08x\n", ioread32(clk));
   clk_read |= 0x5a000000 | 1<<4; 
//   printk("clk_start_changed= 0x%08x\n",clk_read);
   writel( clk_read, clk);
}
//
static void clk_stop(void)
{
   unsigned int clk_read;
   clk_read= ioread32(clk);
//   printk("clk_stop= 0x%08x\n", ioread32(clk));
   clk_read |= 0x5a000000; 
   clk_read &= ~(1<<4); 
//   printk("clk_stop_changed= 0x%08x\n",clk_read);
   writel( clk_read, clk);
}
//
static void cleanup_gpclk (void)
{
 printk ("clean gpclk\n");
 CLK_CTL_KILL = 1<<5 ;
 writel(CLK_PASSWD | CLK_CTL_MASH | CLK_CTL_KILL,clk); 
 iounmap (clk);
}
//
static void delay_sig (void __iomem *registers, unsigned int delay)
{
      volatile unsigned int i,j;
      i=(unsigned long int) ioread32(registers + REG_COUNTER_LO  );
      j=i; 
      while( j < (i+delay))
      {
         j= (unsigned long int) ioread32(registers + REG_COUNTER_LO);
      }
//   printk("counterwhile j = %u \n",j); 
//   printk("counterwhile i = %u \n",i); 
//   printk("counterwhile = %u \n",(j-i)); 
}
// 
static irqreturn_t bcm2835timer_isr(int irq, void *dev)
{
   CountEntry = ioread32(registers + REG_COUNTER_LO  ); 
   while ((CountEntry-CountExit) < INTER) {
      CountEntry = ioread32(registers + REG_COUNTER_LO  ); 
   }
//   printk("counter_entry = %u\n",CountEntry); 
   clk_stop();
//   printk("counter_clk_stop = %u\n",ioread32(registers + REG_COUNTER_LO  )); 
   // Acknoledge the interupt
   writel(1<< DEFAULT_TIMER   , registers); // 
   writel(1 << BLANK,gpio_reg +  GPSET0);
   pinstate = ioread32(gpio_reg + GPLEV0); // GPLEV0 pour les GPIO de 0 a 31 sinon GPLEV1
  
//   
   if ((pinstate & (1 << CALCUL)) != 0)
   {
      writel(1 << XLAT,gpio_reg   +  GPSET0);
      delay_sig(registers,50);
      writel(1 << CALCUL,gpio_reg +  GPCLR0);
      writel(1 << XLAT,gpio_reg   +  GPCLR0);
   }
   else
   {
//      writel(1 << CALCUL,gpio_reg   +  GPSET0); ligne juste pour le test fct assuree par le programe de calcul
      delay_sig(registers,50);
   }
   writel(1 << BLANK,gpio_reg +  GPCLR0);
   // Increment the counter
   writel((INTER-20) +ioread32(registers + REG_COUNTER_LO  ), registers + REG_COMPARE(DEFAULT_TIMER)); // 0x1000 = 4096
   clk_start();
   CountExit=ioread32(registers + REG_COUNTER_LO  );
//   printk("counter_exit = %u\n",CountExit); 
   //
return IRQ_HANDLED;
}
//
/* Table of "compatible" values to search for */
static const struct of_device_id bcm2835timer_of_match[] = {
   { .compatible = "brcm,bcm2835-system-timer" },
   {},
};
/* Platform device info */
static struct platform_driver bcm2835timer_driver = {
   .driver = {
      .name = "bcm2835timer",
      .owner = THIS_MODULE,
      .of_match_table = bcm2835timer_of_match,
   },
   .remove = bcm2835timer_remove,
   .probe = bcm2835timer_probe, 
};
//
static int __init bcm2835timer_probe(struct platform_device *pdev)
{
   struct device *dev = &pdev->dev;
//   struct device_node *np = dev->of_node;
   int rc = 0;
   struct device_node *np;
   u32 value;
//
   CountExit=0;
   CountEntry=0;
// 
   rc = of_address_to_resource(pdev->dev.of_node, 0, &res);
   printk("rc = of_address_to_resource \n");
//
   if (rc) {
       /* Fail */
       printk("fail to access driver in device tree \n");
       rc = -ENOENT;
       platform_driver_unregister(&bcm2835timer_driver);
       return rc;
   }
//
   if (!request_mem_region(res.start, resource_size(&res), "bcm2835timer")) {
       printk("fail to access mem region \n");
       rc = -EBUSY;
       platform_driver_unregister(&bcm2835timer_driver);
       return rc;
   }
   printk("res size= 0x%08x\n", resource_size(&res));
   printk("res.start= x%08x\n", res.start);
   /* recupere registre du timer */
//
   registers = of_iomap(pdev->dev.of_node, 0);
   if (!registers) {
       printk("fail get timer register \n");
       rc = -ENOMEM;
       release_mem_region(res.start, resource_size(&res));
       platform_driver_unregister(&bcm2835timer_driver);
       return rc;
   }
   /* recupere registre des gpio */
//
   np = of_find_compatible_node(NULL, NULL, "brcm,bcm2835-gpio");
   gpio_reg = of_iomap(np, 0);
   if (!gpio_reg) {
      return -ENOMEM;
   }       
//  init de gpclk sur pin 4
   init_gpclk (gpio_reg);
   /* initialise gpio pinnum  */
   init_gpio(BLANK);
   init_gpio(XLAT);
   init_gpio(VPRG);
   init_gpio(CALCUL);
//      
   writel(1 << VPRG,gpio_reg   +  GPCLR0); // set VPRG to low
   /* recupere la valeur irq du timer */
//
   irq = irq_of_parse_and_map(dev->of_node, DEFAULT_TIMER   );
   rc = request_irq(irq, bcm2835timer_isr, 0, DRIVER_NAME, dev);
   printk("irq = %u\n",irq); 
   /* affichage pour debug */
//
   value = ioread32(registers);
   printk("irq_reg = 0x%08x\n", (unsigned int)  registers);
   printk("gpio_reg = 0x%08x\n", (unsigned int)  gpio_reg);
   printk("counter = %u\n",ioread32(registers + REG_COUNTER_LO  ));
   /* ecriture initiale de la valeur de comparaison du compteur */
   writel(INTER +ioread32(registers + REG_COUNTER_LO  ), registers + REG_COMPARE(DEFAULT_TIMER));
//   clk_start();
// 
   return 0;
}
static int bcm2835timer_remove(struct platform_device *pdev)
{
   dev_info(&pdev->dev, "remove\n");
   free_irq(irq, &pdev->dev);
   iounmap(registers);
   iounmap(gpio_reg);
   release_mem_region(res.start, resource_size(&res));
   return 0;
} 
//
static int __init bcm2835timer_init(void)
{
   pr_info(DRIVER_NAME ": init\n");
   return platform_driver_register(&bcm2835timer_driver);
}
//
static void __exit bcm2835timer_exit(void)
{
   cleanup_gpclk();
   platform_driver_unregister(&bcm2835timer_driver);
   pr_info(DRIVER_NAME ": exit\n");
}
// 
module_init(bcm2835timer_init);
module_exit(bcm2835timer_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Nicolas Coustaud");
MODULE_DESCRIPTION("Pi3 bcm2835 timer");
MODULE_DEVICE_TABLE(of, bcm2835timer_of_match);
