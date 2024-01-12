#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#define RECV_BUUF_SIZE 64
#define SERVER_PORT 37260
#define SERVER_IP "192.168.144.25"
// Gimbal Camera (Server) Port
// Gimbal Camera (Server) IP Addresses
int main(int argc, char *argv[]) {
  int sockfd;
  int ret, i, recv_len;
  struct sockaddr_in send_addr, recv_addr;
  unsigned char send_buf[] = {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x01, 0xd1, 0x12};
  // Frame protocol of the relevant functions in hexadecimal
  unsigned char recv_buf[RECV_BUUF_SIZE] = {0};
  /* Create UDP Socket
  AF_INET:
  ipv4 addresses
  SOCK_DGRAM: UDP protocol
  0:
  automatically choose the default protocol of the relevant type
  */
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("socket");
    exit(1);
  }
  /* Set IP addresses and port number of gimbal camera
  sin_family:
  ipv4 addresses
  sin_addr.s_addr: IP addresses of gimbal camera
  sin_port：
  port of gimbal camera
  */
  memset(&send_addr, 0, sizeof(send_addr));
  send_addr.sin_family = AF_INET;
  send_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
  send_addr.sin_port = htons(SERVER_PORT);
  /* Send frame data
  sockfd:
  descriptor of socket
  send_buf：
  head address in RAM of the sending data
  sizeof(send_buf)：
  length of sending data
  0：
  sending mark, usually it is 0
  (struct sockaddr *)&send_addr:
  structure pointer of the receiving data addresses
  (including IP addresses and port)
  addr_len:
  structure size of the receiving data addresses
  */
  printf("Send HEX data\n");
  socklen_t addr_len = sizeof(struct sockaddr_in);
  if (sendto(sockfd, send_buf, sizeof(send_buf), 0, (struct sockaddr *) &send_addr, addr_len) < 0) {
    perror("sendto");
    exit(1);
  }
  /* Receive the responding data from gimbal camera
  sockfd:
  descriptor of “sockfd” socket
  recv_buf:
  head address in RAM of the responding data
  RECV_BUUF_SIZE:
  size of the buffer, which is the length of the max data to
  receive
  0:
  receiving mark, usually it is 0
  (struct sockaddr *)&recv_addr:
  the target structure will be filled with addresses (including
  IP addresses and port) from the data sender
  &addr_len:
  the target storage position, the structure size of
  “src_addr” and “addrlen” should be filled before calling, the actual size of the sender will be filled after calling
  */
  recv_len = recvfrom(sockfd, recv_buf, RECV_BUUF_SIZE, 0, (struct sockaddr *) &recv_addr,
                      &addr_len);
  if (recv_len < 0) {
    perror("recvfrom");

    exit(1);
  }
  // print the received data in hexadecimal
  printf("Received HEX data: ");
  for (int i = 0; i < recv_len; i++) {
    printf("%02x ", recv_buf[i]);
  }
  printf("\n");
  // close socket
  close(sockfd);
  return 0;
}
