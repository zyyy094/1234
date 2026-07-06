/*
 * rpmsg_handler.h - RPMsg 通信处理
 *
 * 从核端 RPMsg 端点管理，接收主核命令并分发到硬件控制
 */
#ifndef RPMSG_HANDLER_H
#define RPMSG_HANDLER_H

/* 初始化 RPMsg 通信 */
int rpmsg_handler_init(void);

/* 运行 RPMsg 消息循环（阻塞，收到 shutdown 才退出） */
int rpmsg_handler_run(void);

/* 清理 RPMsg 资源 */
void rpmsg_handler_cleanup(void);

#endif /* RPMSG_HANDLER_H */
