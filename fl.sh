#!/bin/bash

if [ $# -lt 1 ]
then
  echo 'please input pid：'
  echo './xxx.sh [pid] [option:filename] [option:sleepTime] [option:dstPrefixPath]'
  exit 1
fi

# 默认参数
flameGraphPath=/home/jon/workspace/FlameGraph
dstPrefixPath=/home/jon/workspace/tmp
sleepTime=30
filename=test

if [[ $2 && -n "$2" ]]
then
  filename=$2
fi
if [[ $3 && $3 -gt 0 ]]
then
  sleepTime=$3
fi
if [[ $4 && -n "$4" ]]
then
  dstPrefixPath=$4
fi

sudo perf record -g -F 200 -p $1 -o $dstPrefixPath/$filename.data -- sleep $sleepTime
sudo perf script -i $dstPrefixPath/$filename.data > $dstPrefixPath/$filename.perf
$flameGraphPath/stackcollapse-perf.pl $dstPrefixPath/$filename.perf > $dstPrefixPath/$filename.floded
$flameGraphPath/flamegraph.pl $dstPrefixPath/$filename.floded > $dstPrefixPath/$filename.svg
