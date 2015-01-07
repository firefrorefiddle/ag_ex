#!/bin/sh

tar cz * | ssh alg0828656@eowyn.ads.tuwien.ac.at -C "tar xz; make; ./tcbvrp"
