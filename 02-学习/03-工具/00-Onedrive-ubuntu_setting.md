
## 用到的命令
```bash
onedrive --display-config
vi /home/guohw/.config/onedrive/config
onedrive --monitor --verbose --confdir="~/.config/onedrive"
```
## Config设置
```bash
skip_file = ".git/* "
skip_symlinks = "true"
monitor_interval = "300"
skip_dotfiles = "true"
```

## 其他
onedrive --monitor vs onedrive  --synchronize
前面的命令可以在后台运行，后面的命令是直接运行
