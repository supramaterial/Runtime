/*
 * Copyright (c) 2014 ELL-i co-operative.
 *
 * This file is part of ELL-i software.
 *
 * ELL-i software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ELL-i software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ELL-i software.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Minimal standalone UDP
 *
 * @author: Pekka Nikander <pekka.nikander@ell-i.org>  2014
 */

#include <ip.h>
#include <udp.h>

/**
 * XXX
 */

#ifdef EMULATOR
#include <stdio.h>
#define error(...) fprintf(stderr, __VA_ARGS__)
#else
#define error(...)
#endif

void udp_input(struct udp *const udp_packet) {
#if 0
    /*
     * Verify UDP checksum
     */
    XXX;
#endif

    /* Swap ports */
    register uint16_t dport = udp_packet->udp_dport;
    udp_packet->udp_dport = udp_packet->udp_sport;
    udp_packet->udp_sport = dport;

    /* Find upper layer handler, if any */
    for (const struct udp_socket *socket = __udp_sockets; 
         socket < __udp_sockets_end;
         socket++ /* XXX */) {
        if (socket->udp_port == dport) {
            /* XXX Update statistics */
            socket->udp_socket_handler(udp_packet, udp_packet->udp_data, udp_packet->udp_len);
            return;
        }
    }
    error("Packet to unbound UDP socket %d\n", dport);
}

/**
 * XXX
 */

void udp_output(struct udp *const udp, uint16_t len) {
    len += sizeof(struct udp);

    udp->udp_len = htons(len);
    
    /*
     * Clear the checksum, for now
     */
    udp->udp_sum = 0;
    /*
     * Pass to lower layer.
     */
    struct ip *const ip = (struct ip *)(((char *)udp) - sizeof(struct ip));
    ip_output(ip, len + sizeof(struct ip));
}

