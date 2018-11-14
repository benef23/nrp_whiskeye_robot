#!/bin/bash
find . -name "*.py" -exec sed -i.orig 's/    /\t/g' {} +

