
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.Virgule_pkg.all;
use work.Computer_pkg.all;

entity Computer is
    port(
        clk_i        : in  std_logic;
        btn_center_i : in  std_logic;
        switches_i   : in  std_logic_vector(15 downto 0);
        leds_o       : out std_logic_vector(15 downto 0);
        uart_rx_i    : in std_logic;
        uart_tx_o    : out std_logic
    );
end Computer;

architecture Structural of Computer is
    signal sync_reset    : std_logic;
    signal sync_switches : std_logic_vector(15 downto 0);

    signal core_valid    : std_logic;
    signal core_ready    : std_logic;
    signal core_address  : word_t;
    signal core_rdata    : word_t;
    signal core_wdata    : word_t;
    signal core_write    : std_logic_vector(3 downto 0);
    signal core_irq      : std_logic;

    alias dev_address    : byte_t is core_address(31 downto 24);

    signal mem_valid     : std_logic;
    signal mem_ready     : std_logic;
    signal mem_rdata     : word_t;

    signal io_valid      : std_logic;
    signal io_ready      : std_logic;
    signal io_rdata      : word_t;
    
    signal sync_uart_rx  : std_logic;
    signal uart_ready    : std_logic;
    signal uart_valid    : std_logic;
    signal uart_rdata    : word_t;
    
    signal intc_valid    : std_logic;
    signal intc_ready    : std_logic;
    signal intc_rdata    : word_t;
    signal intc_events   : word_t;
    
    signal timer_valid    : std_logic;
    signal timer_ready    : std_logic;
    signal timer_rdata    : word_t;
       
    signal uart_tx_evt    : std_logic;
    signal uart_rx_evt    : std_logic;
    signal timer_evt      : std_logic;
    


begin
    -- Concurrent statements
    core_inst : entity work.Virgule
        port map(
            clk_i => clk_i,
            reset_i => sync_reset,
            irq_i => core_irq,
            rdata_i => core_rdata,
            wdata_o => core_wdata,
            write_o => core_write,
            address_o => core_address,
            ready_i => core_ready,
            valid_o => core_valid
        );
        
     mem_inst : entity work.VMemory(Behavioral)
        generic map(
            CONTENT => MEM_CONTENT
        )
        port map(
            clk_i => clk_i,
            reset_i => sync_reset,  
            valid_i => mem_valid,  
            ready_o => mem_ready,
            address_i => core_address(31 downto 2),
            wdata_i => core_wdata,
            write_i => core_write,
            rdata_o => mem_rdata
    );
        
    sync_inst : entity work.InputSynchronizer(Behavioral)
        generic map(
            WIDTH => 18 
        )
        port map(
            clk_i => clk_i,
            data_i(17) => uart_rx_i,
            data_i(16 downto 1) => switches_i,
            data_i(0) => btn_center_i,
            data_o(16 downto 1) => io_rdata(15 downto 0),
            data_o(0) => sync_reset,
            data_o(17) => sync_uart_rx
        );
        
    io_rdata(31 downto 16) <= (others => '0');
        
      uart_inst : entity work.UART(Structural)
      generic map(
        CLK_FREQUENCY_HZ => CLK_FREQUENCY_HZ,
        BIT_RATE_HZ => UART_BIT_RATE_HZ
      )
      port map(
        clk_i => clk_i,
        reset_i => sync_reset,
        valid_i => uart_valid,
        ready_o => uart_ready,
        write_i => core_write(0),
        wdata_i => core_wdata(7 downto 0),
        rdata_o => uart_rdata(7 downto 0),
        tx_o => uart_tx_o,
        rx_i => sync_uart_rx,
        tx_evt_o => uart_tx_evt,
        rx_evt_o => uart_rx_evt
       );

    intc_events(INTC_EVENTS_UART_RX) <= uart_rx_evt;
    intc_events(INTC_EVENTS_UART_TX) <= uart_tx_evt;
    intc_events(INTC_EVENTS_TIMER) <= timer_evt;
    intc_events(31 downto 3) <= (others => '0');
       
    uart_rdata(31 downto 8) <= (others => '0');
       
    intc_inst : entity work.VInterruptController(Behavioral)
    port map(
        clk_i => clk_i,
        reset_i => sync_reset,
        valid_i => intc_valid,
        ready_o => intc_ready,
        address_i => core_address(2),
        write_i => core_write,
        wdata_i => core_wdata,
        rdata_o => intc_rdata,
        irq_o => core_irq,
        events_i => intc_events
       );
  
    timer_inst : entity work.Timer(Behavioral)
    port map(
        clk_i => clk_i,
        reset_i => sync_reset,
        valid_i => timer_valid,
        ready_o => timer_ready,
        address_i => core_address(2),
        write_i => core_write,
        wdata_i => core_wdata,
        rdata_o => timer_rdata,
        evt_o => timer_evt
        );
      
      
    
    p_mem_valid : process(dev_address, core_valid)
    begin    
        if (dev_address = MEM_ADDRESS) then
            mem_valid <= core_valid;
        else
            mem_valid <= '0';
        end if;
    end process p_mem_valid;
    
    
    p_uart_valid : process(dev_address, core_valid)
    begin    
        if (dev_address = UART_ADDRESS) then
            uart_valid <= core_valid;
        else
            uart_valid <= '0';
        end if;
    end process p_uart_valid;
    
    
    p_intc_valid : process(dev_address, core_valid)
    begin    
        if (dev_address = INTC_ADDRESS) then
            intc_valid <= core_valid;
        else
            intc_valid <= '0';
        end if;
    end process p_intc_valid;
    
    
    p_io_valid : process(dev_address, core_valid)
    begin
        if (dev_address = IO_ADDRESS) then
            io_valid <= core_valid;
        else
            io_valid <= '0';
        end if;
    end process p_io_valid;
    
    p_timer_valid : process(dev_address, core_valid)
    begin
        if (dev_address = TIMER_ADDRESS) then
            timer_valid <= core_valid;
        else
            timer_valid <= '0';
        end if;
    end process p_timer_valid;
    
    
    p_core_rdata : process(dev_address, mem_rdata, io_rdata, uart_rdata, intc_rdata, timer_rdata)
    begin
        if (dev_address = MEM_ADDRESS) then
            core_rdata <= mem_rdata;
        elsif (dev_address = IO_ADDRESS) then
            core_rdata <= io_rdata;
        elsif (dev_address = UART_ADDRESS) then
            core_rdata <= uart_rdata;
        elsif (dev_address = INTC_ADDRESS) then
            core_rdata <= intc_rdata;
        elsif (dev_address = TIMER_ADDRESS) then
            core_rdata <= timer_rdata;
        else 
            core_rdata <= (others => '0');
        end if;
    end process p_core_rdata;
    
    
    p_leds_o_a : process(clk_i, core_write, io_valid)
    begin
        if rising_edge(clk_i) then
            if (core_write(0) = '1' and io_valid = '1') then
                leds_o(7 downto 0) <= core_wdata(7 downto 0);
            end if;
        end if;
    end process p_leds_o_a;
    
    
    p_leds_o_b : process(clk_i, core_write, io_valid)
    begin
        if rising_edge(clk_i) then
            if (core_write(1) = '1' and io_valid = '1') then
                leds_o(15 downto 8) <= core_wdata(15 downto 8);
            end if;
        end if;
    end process p_leds_o_b;
    
    
    p_core_ready : process(dev_address, core_valid, mem_ready, intc_ready, uart_ready, timer_rdata)
    begin
        if dev_address = MEM_ADDRESS then
            core_ready <= mem_ready;
        elsif dev_address = UART_ADDRESS then
            core_ready <= uart_ready;
        elsif dev_address = INTC_ADDRESS then
            core_ready <= intc_ready;
        elsif dev_address = TIMER_ADDRESS then
            core_ready <= timer_ready;
        else
            core_ready <= core_valid;
        end if;
    end process p_core_ready;
    
    
end Structural;
