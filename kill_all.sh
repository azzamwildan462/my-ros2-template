#!/bin/bash

kill -9 $(ps ax | grep 'ros' | awk '{print $1}') 2> /dev/null
