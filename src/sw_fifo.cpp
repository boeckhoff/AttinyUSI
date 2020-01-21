#include <sw_fifo.h>
 
 
sw_fifo_typedef rx_fifo = { {0}, 0, 0, 0 }; // declare a receive software buffer
sw_fifo_typedef tx_fifo = { {0}, 0, 0, 0 }; // declare a transmit software buffer

 
void uart_byte_received(uint8_t data) {
   
  if(rx_fifo.num_bytes == FIFO_BUFFER_SIZE) {      // if the sw buffer is full
    uart_rx_fifo_ovf_flag = 1;                     // set the overflow flag
  }else if(rx_fifo.num_bytes < FIFO_BUFFER_SIZE) { // if there's room in the sw buffer
     
     
    rx_fifo.data_buf[rx_fifo.i_last] = data;
     
    rx_fifo.i_last++;                              // increment the index of the most recently added element
    rx_fifo.num_bytes++;                           // increment the bytes counter
  }
  if(rx_fifo.num_bytes == FIFO_BUFFER_SIZE) {      // if sw buffer just filled up
    uart_rx_fifo_full_flag = 1;                    // set the RX FIFO full flag
  }
  if(rx_fifo.i_last == FIFO_BUFFER_SIZE) {         // if the index has reached the end of the buffer,
    rx_fifo.i_last = 0;                            // roll over the index counter
  }
  uart_rx_fifo_not_empty_flag = 1;                 // set received-data flag
}