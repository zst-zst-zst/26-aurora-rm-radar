int8 self_color #0是蓝 2是红
int16 match_time # 比赛时间，若尚未开始则发送比赛开始倒计时时间的负数，若比赛结束发送-100 若未连接至裁判系统发送-200
uint8 game_progress # 0x0001 game_progress字段: 0未开始 1准备阶段(有人) 2自检(场地已清空) 3倒计时 4比赛中 5结束
uint16[16] robot_hp #参考0x0003, uint16 以支持英雄等血量>255的机器人
uint8[5] marks  # slot: 0=hero 1=engineer 2=infantry3 3=infantry4 4=sentry
uint8 ultimate
uint32 eventtype
uint8 dart_remaining_time # 参考0x0105，己方飞镖发射剩余时间
uint8 dart_last_target # 参考0x0105，最近一次己方飞镖击中的目标
uint8 dart_hit_count # 参考0x0105，对方最近被击中的目标累计被击中次数
uint8 dart_selected_target # 参考0x0105，飞镖当前选定的击打目标
