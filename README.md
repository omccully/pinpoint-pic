# Pinpoint PIC16LF1827 Program

## Program Explanation from Final Report

The program for the PIC16LF1827 microcontroller is used to unify and control all components of the Pinpoint device. When the program starts, various registers are configured. Since the HI-TECH C Compiler libraries do not provide a software UART, a software UART transmitter was written to send commands to the GPS chip over port RB3 after the initial register configurations are made. Two commands are sent to the GPS chip. One command configures the GPS chip to send update messages every 5 seconds, and the other command configures the GPS chip to only send NMEA messages that include coordinate information. The hardware UART RX port is used to receive the acknowledgments from the GPS chip to ensure that the GPS chip was successfully configured. Once the GPS chip is configured, interrupts are enabled to enable the rest of the functionality of the program. 

The external interrupt (INT) was configured to occur when the normally-low pushbutton connected to port RB0 was pressed. When this interrupt occurs, the message “@PANIC-B-0” is sent out the hardware UART to the transmitter. The “B” stands specifies that it is an alert generated due to the button press, and the “0” specifies that it occurred 0 seconds ago. The Timer0 module is configured with a prescaler of 256, and Timer0’s overflow interrupt is enabled to interrupt about every 0.524 seconds. The Timer0 overflow interrupt is used to repeat alerts approximately every 5 seconds until it has been repeated 12 times or another alert occurred. 

The comparator module within the PIC16LF1827 microcontroller is used to trigger an interrupt once the microphone volume exceeds a setpoint. The comparator module is configured to have one input be a reference voltage generated from a DAC, and the other input to the comparator is the microphone voltage on port RA0. Once the comparator interrupt occurs, a panic message “@PANIC-M-0” is sent to the transmitter. The panic message repeating system is used to repeat this alert in the same manner as the button alert.

In the main loop of the program, the three accelerometer axes are read from the A/D converter, and the magnitude of acceleration is determined. The sum of the magnitudes is calculated and compared to a set value. If the sum of the magnitudes exceeds the set value, then the panic message “@PANIC-A-0” is sent to the transmitter. The panic message repeating system is used to repeat this alert in the same manner as the button alert. It must be noted that this sum of magnitudes is not an accurate method of determining the device’s magnitude of acceleration. 

The baud rate for the hardware UART is set to 9615 bps, which is the closest possible baud rate to 9600 with a 500 KHz clock on the PIC16LF1827. The hardware RX is connected to the TX of the GPS chip to receive NMEA strings. The biggest challenge when developing this program is dealing with having different messages coming from multiple sources (incoming NMEA strings, generated panic signals) that must be sent out the same TX pin. Considerations have to be made for special cases, such as determining what occurs if a panic condition occurs while transmitting an NMEA string to the wireless transmitter. With a baud rate of 9600 bps, this is not uncommon scenario. To solve this, three different character array buffers are used: an NMEA string receive buffer, a alert message buffer, and a buffer used to send data. The RCIF interrupt is used to receive bytes one at a time and place them in the nmea_buffer character array. Once the NMEA string has been completely read into the nmea_buffer array, the nmea_buffer array is copied into the send_buffer array, or if something is currently being sent on the TX pin, the nmea_buffer_ready boolean variable will be set to true. The send_buffer_locked boolean variable is set to true until all characters in send_buffer have been sent. The send_buffer_locked variable ensures nothing is written over the message in send_buffer before it is completely sent. Setting nmea_buffer_ready to true specifies that there is a complete message in the nmea_buffer, so it is ready to be sent (moved into send_buffer) as soon as the message in send_buffer finishes being sent. A similar mechanism is used to send alert messages with the alert_buffer and alert_buffer_ready variables. If both an NMEA string and a panic message are ready to be sent, the panic message will take priority. It is important that the start_panicing function is an atomic operation, so interrupts are disabled before calling it and after calling it in the main loop. Interrupts do not have to be disabled before the other calls to the start_panicing function because they are called within an interrupt, and interrupts cannot occur within interrupts on the PIC16LF1827. Interrupts occurring within the start_panicing function could cause a corruption of the alert_buffer, send_buffer, and the related alert_buffer_ready and send_buffer_locked boolean values. 