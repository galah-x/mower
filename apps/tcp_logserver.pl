#!/usr/bin/perl
use strict;
use warnings;

# This is a trivial cow logfile server to write the log from the SD card from th eepanel gadget to a local file.
# listen to a tcp socket, copy to a file on open.     Repeat. 
# format is as in the source file.

use IO::Socket;
use POSIX qw(strftime);

my $port = 5681;
my $socket = IO::Socket::INET->new(LocalPort => $port,
				   Proto => "tcp",
				   Listen => 5
    )  or die "Can't set up tcp server on port $port\n";

my $fh;
my $size;


my $filenameroot = "/tmp/mowerlog.txt";
while (1)
{
    my $client_socket = $socket->accept();
        # get information about a newly connected client
    my $client_address = $client_socket->peerhost();
    my $client_port = $client_socket->peerport();
    print "connection from $client_address:$client_port\n";

    #  I'm a fileserver to write the connected gadget cow logfile. so just get the file.
    my $ldate = strftime "%04Y%02m%02d%H%M%S", localtime();
    open $fh, "> $filenameroot.$ldate" or die "cannot open $filenameroot.$ldate for write\n";

    my $data = "";
    $size=0;

    # grab data 1k at a time while the socket is open
#    while ($client_socket->recv($data, 1024)) {
    while ($data=<$client_socket>) {
	print $fh $data;
	$size += length($data);
	$data = '';
    }
    close $fh;
    shutdown($client_socket,1);
    print "received $size bytes to $filenameroot.$ldate\n";
}

