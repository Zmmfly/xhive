/* Compile/link fixture only; no flashing, hardware run or motor qualification. */
int fieldctrl_probe_linears(void);
int fieldctrl_probe_filters(void);
int fieldctrl_probe_control(void);

volatile int fieldctrl_compile_result;

/* Newlib calls these hooks with -nostartfiles. This C-only fixture has no
 * additional runtime initialization/finalization; SDK startup handles memory. */
void _init(void) {}
void _fini(void) {}

int main(void) {
    int result = fieldctrl_probe_linears();
    result |= fieldctrl_probe_filters();
    result |= fieldctrl_probe_control();
    fieldctrl_compile_result = result;
    return result;
}
