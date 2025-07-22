#ifndef LWIPOPTS_H
#define LWIPOPTS_H

// Configurações básicas
#define NO_SYS 1
#define LWIP_SOCKET 0
#define LWIP_NETCONN 0

// Protocolos habilitados
#define LWIP_TCP 1
#define LWIP_UDP 1
#define LWIP_IPV4 1
#define LWIP_ICMP 1
#define LWIP_DHCP 1
#define LWIP_AUTOIP 0  // Desabilitado para economizar memória
#define LWIP_DNS 1
#define LWIP_RAW 0     // Desabilitado pois não estamos usando raw sockets

// Configurações de memória (aumentadas significativamente)
#define MEM_ALIGNMENT 4
#define MEM_SIZE (12 * 1024)            // Aumentado para 12KB
#define MEMP_NUM_PBUF 16
#define PBUF_POOL_SIZE 16               // Número de buffers PBUF
#define PBUF_POOL_BUFSIZE 512           // Tamanho de cada buffer PBUF
#define MEMP_NUM_TCP_PCB 5              // Número de conexões TCP simultâneas
#define MEMP_NUM_TCP_SEG 32             // Número de segmentos TCP
#define MEMP_NUM_UDP_PCB 4              // Número de conexões UDP

// Configurações do TCP
#define TCP_WND (4 * TCP_MSS)           // Janela TCP
#define TCP_MSS (1460)                  // Maximum Segment Size
#define TCP_SND_BUF (4 * TCP_MSS)       // Buffer de envio TCP
#define TCP_SND_QUEUELEN (8 * TCP_SND_BUF / TCP_MSS)

// Configurações HTTP (otimizadas para seu caso)
#define LWIP_HTTPD 1
#define LWIP_HTTPD_SSI 0                // Desabilitado SSI (não está usando)
#define LWIP_HTTPD_SUPPORT_POST 0       // Desabilitado POST (não está usando)
#define LWIP_HTTPD_CGI 0                // Desabilitado CGI (não está usando)
#define LWIP_HTTPD_DYNAMIC_HEADERS 1    // Necessário para seu servidor
#define HTTPD_USE_CUSTOM_FSDATA 0       // Usando sistema de arquivos padrão

// Outras otimizações
#define LWIP_NETIF_HOSTNAME 1
#define LWIP_STATS 0                    // Desabilitar estatísticas para economizar memória
#define LWIP_DEBUG 0                    // Desabilitar debug em produção

// Timeouts (ajustados para conexões HTTP)
#define TCP_LISTEN_BACKLOG 1
#define TCP_DEFAULT_LISTEN_BACKLOG 1

#endif /* LWIPOPTS_H */