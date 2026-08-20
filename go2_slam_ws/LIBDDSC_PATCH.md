# libddsc.so 崩溃修复补丁(2026-08-10)

Unitree 定制 CycloneDDS 0.10.2(no-SHM)的 xt_validate_impl(ddsi_typewrap.c)
在 t 为小整数 tag 值(如 0x51,dq.builtins 线程调用)时会解引用崩溃 SEGV。

## 补丁内容(相对原版 libddsc.so.bak.20260810)

### 0x8f1b0(xt_validate_impl 入口,24 字节)
原: stp x29,x30,[sp,#-80]! / tst w2,#0xff / ...
改:
```
8f1b0: f104003f  cmp x1, #0x100        ; t 是大指针?
8f1b4: 54000069  b.ls 8f1c0            ; 小值(tag)→ 直接返回 0,不碰内存
8f1b8: 52800a23  mov w3, #0x51         ; 大指针(xt)→ tag=0x51
8f1bc: 39013023  strb w3, [x1, #76]    ; xt->tag = 0x51(XTypes)
8f1c0: 52800000  mov w0, #0            ; 返回 0(成功)
8f1c4: d65f03c0  ret
```
字节: 3f0004f1 69000054 230a8052 23300139 00008052 c0035fd6

### 0x90128-0x90133(已恢复原始,ddsi_xt_type_init_impl 主路径)
```
90128: aa1303e1  mov x1, x19
9012c: aa1603e0  mov x0, x22
90130: 97fe64e0  bl 294b0 <ddsi_xt_validate@plt>
```
字节: aa1303e1 aa1603e0 e064fe97
(之前会话误将 validate 调用改掉导致 busy loop——假成功,类型 tag 永不写入)

## 验证通过(2026-08-10)
- 三进程存活: xt16_driver / unitree_slam / web_server
- CPU: xt16_driver ~15%, unitree_slam ~10%, web_server ~70%(回落)
- 雷达 /unitree/slam_lidar/points 10Hz
- web 控制台 http://<IP>:8890 HTTP 200
- lego_loam scan2map LM 循环正常

## 备份/回滚
- 原版: /usr/local/lib/libddsc.so.bak.20260810
- 补丁版(验证通过): /home/unitree/libddsc.so.patched_valid_20260810.bak + /usr/local/lib/libddsc.so.patched_valid_20260810.bak
- 两处库必须同步打补丁: /usr/local/lib 与 go2_slam_ws/module/unitree_slam/lib
- 回滚: sudo cp libddsc.so.bak.20260810 /usr/local/lib/libddsc.so
