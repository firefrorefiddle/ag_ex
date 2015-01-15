#!/bin/sh

tar cz --exclude=ins_visual --exclude=Visualizer * | ssh alg0828656@eowyn.ads.tuwien.ac.at -C "tar xz && make && ./tcbvrp && echo && echo Model: && echo && cat model.lp"
