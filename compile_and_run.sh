#!/bin/sh

tar cz * | ssh alg1225049@eowyn.ads.tuwien.ac.at -C "tar xz; make; ./tcbvrp"
