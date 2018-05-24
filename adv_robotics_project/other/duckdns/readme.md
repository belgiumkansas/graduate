#DuckDNS Setup

##commands to run

- `cd ~`
- `mkdir duckdns`
- cd `duckdns`
- copy the duck.sh into this directory
- `chmod 700 duck.sh`
-`crontab -e`
- add this to the bottom of crontab `*/5 * * * * ~/duckdns/duck.sh >/dev/null 2>&1`
- `./duck.sh` 
- `cat duck.log` to test. "OK" indicates it's good.

