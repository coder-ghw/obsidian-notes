
- 挂载nas目录
```bash
sudo mount -t cifs -o uid=<host账号>,username=<nas账号>,password=<nas密码>,iocharset=utf8 <server dir> <host dir>
```