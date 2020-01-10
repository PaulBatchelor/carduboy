void init(void);
void setup(void);
void loop(void);
void USBDevice_attach(void);

int main(void)
{
    init();

    USBDevice_attach();

    setup();

    while(1) loop();

    return 0;
}
