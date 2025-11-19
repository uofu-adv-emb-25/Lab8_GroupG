# Lab 8
Lab 8 for Luke and James

## Observations
We used an id of 1 with a 1ms delay on one board and an id of 4 with a 1000ms delay on the other. We found that the low priority broadcasted the expected amount: ~10 transmissions for 10,000 total transmissions, likely because there is enough delay for the low priority to fit in becasue the transission took about 200us. At low delays, the low priority message wouldn't fit in.