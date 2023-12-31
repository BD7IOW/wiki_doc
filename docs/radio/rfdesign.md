# 一、FastDFS入门

## 应用场景

互联网海量非结构化数据的存储需求

- 电商网站：海量的商品图片
- 视频网站：海量视频文件
- 网盘：海量文件
- 社交网站：海量图片

> 分布式文件系统（DFS）是一个软件/软件服务器，这个软件可以用来管理文件。但是这个软件所管理的文件通常不是在一个服务器节点上，而是在多个服务器节点上，这些服务器节点通过网络相连构成一个庞大的文件存储服务器集群，这些服务器都用于存储文件资源，通过分布式文件系统来管理这些服务器上的文件。

![1632561800935](../images/1632561800935.png)

> 分布式文件系统与传统文件系统对比

![1632562470548](../images/1632562470548.png)

## FastDFS简介

> FastDFS是一个开源的轻量级[分布式文件系统](https://baike.baidu.com/item/分布式文件系统/1250388)，它对文件进行管理，功能包括：文件存储、文件同步、文件访问（文件上传、文件下载）等，解决了大容量存储和[负载均衡](https://baike.baidu.com/item/负载均衡/932451)的问题。特别适合以文件为载体的在线服务，如相册网站、视频网站等等。
>
> FastDFS为互联网量身定制，充分考虑了冗余备份、负载均衡、线性扩容等机制，并注重高可用、高性能等指标，使用FastDFS很容易搭建一套高性能的文件服务器集群提供文件上传、下载等服务。主要解决了海量数据存储问题，特别适合以中小文件（建议范围：4KB < file_size <500MB）为载体的在线服务。 如果是更大的文件，建议使用其他的分布式文件系统更合适。

## 特性

优点：

- 文件不分块存储，文件和系统中的文件一一对应
- 对文件内容做hash处理，避免出现重复文件、节约磁盘空间
- 下载文件支持HTTP协议，可基于内置的Web Server或外部Web Server
- 支持在线扩容，动态添加卷
- 支持文件冗余备份和负载均衡
- 存储服务器可以保存文件属性
- V2.0网路通信采用libevent，支持大并发访问，整体性能更好

缺点：

- 直接按文件存储，可直接查看文件内容，缺乏安全性
- 数据同步无校验，存在静默IO问题，降低系统可用性
- 单线程数据同步，仅适合存储小文件（4KB到500MB之间）
- 备份数据根据存储分卷（分组）决定，缺乏文件备份数设置灵活性
- 单个挂载点异常会导致整个存储节点下线
- 缺房多机房容灾支持
- 静态的负载均衡机制

优缺点并存，但是针对中小系统已经完全足够使用

## FastDFS架构

> FastDFS架构包括 Tracker server和Storage server。客户端请求Tracker server进行文件上传、下载，通过Trackerserver调度最终由Storage server完成文件上传和下载。
>
> FastDFS 系统有三个角色：跟踪服务器(Tracker Server)、存储服务器(Storage Server)和客户端(Client)。
>
> 　　Tracker Server：跟踪服务器，主要做调度工作，起到均衡的作用；负责管理所有的 storage server和 group，每个 storage 在启动后会连接 Tracker，告知自己所属 group 等信息，并保持周期性心跳。通过Trackerserver在文件上传时可以根据一些策略找到Storageserver提供文件上传服务。
>
> 　　Storage Server：存储服务器，主要提供容量和备份服务；以 group 为单位，每个 group 内可以有多台 storage server，数据互为备份。Storage server没有实现自己的文件系统而是利用操作系统 的文件系统来管理文件。
>
> 　　Client：客户端（可以是java程序，也可以是浏览器），上传下载数据的服务器，也就是我们自己的项目所部署在的服务器。

![1632565884091](../images/1632565884091.png)

## Tracker 集群

> ​         FastDFS集群中的Tracker server可以有多台，Trackerserver之间是相互平等关系同时提供服务，Trackerserver不存在单点故障。客户端请求Trackerserver采用轮询方式，如果请求的tracker无法提供服务则换另一个tracker。

## Storage 集群

> 为了支持大容量，存储节点（服务器）采用了分卷（或分组）的组织方式。存储系统由一个或多个卷组成，卷与卷之间的文件是相互独立的，所有卷的文件容量累加就是整个存储系统中的文件容量。一个卷由一台或多台存储服务器组成，卷内的Storage server之间是平等关系，不同卷的Storageserver之间不会相互通信，同卷内的Storageserver之间会相互连接进行文件同步，从而保证同组内每个storage上的文件完全一致的。一个卷的存储容量为该组内存储服务器容量最小的那个，由此可见组内存储服务器的软硬件配置最好是一致的。卷中的多台存储服务器起到了冗余备份和负载均衡的作用
>
> 在卷中增加服务器时，同步已有的文件由系统自动完成，同步完成后，系统自动将新增服务器切换到线上提供服务。当存储空间不足或即将耗尽时，可以动态添加卷。只需要增加一台或多台服务器，并将它们配置为一个新的卷，这样就扩大了存储系统的容量。
>
>  采用分组存储方式的好处是灵活、可控性较强。比如上传文件时，可以由客户端直接指定上传到的组也可以由tracker进行调度选择。一个分组的存储服务器访问压力较大时，可以在该组增加存储服务器来扩充服务能力（纵向扩容）。当系统容量不足时，可以增加组来扩充存储容量（横向扩容）。 

## 上传过程

FastDFS向使用者提供基本文件访问接口，比如upload、download、append、delete等，以客户端库的方式提供给用户使用。

Storage Server会定期的向Tracker Server发送自己的存储信息。当Tracker Server Cluster中的Tracker Server不止一个时，各个Tracker之间的关系是对等的，所以客户端上传时可以选择任意一个Tracker。

当Tracker收到客户端上传文件的请求时，会为该文件分配一个可以存储文件的group，当选定了group后就要决定给客户端分配group中的哪一个storage server。当分配好storage server后，客户端向storage发送写文件请求，storage将会为文件分配一个数据存储目录。然后为文件分配一个fileid，最后根据以上的信息生成文件名存储文件。

![1632566733636](../images/1632566733636.png)

>  客户端上传文件后存储服务器将文件ID返回给客户端，此文件ID用于以后访问该文件的索引信息。文件索引信息包括：组名，虚拟磁盘路径，数据两级目录，文件名。 

![1632567138665](../images/1632567138665.png)

- 组名：文件上传后所在的storage组名称，在文件上传成功后有storage服务器返回，需要客户端自行保存。
- 虚拟磁盘路径：storage配置的虚拟路径，与磁盘选项store_path*对应。如果配置了store_path0则是M00，如果配置了store_path1则是M01，以此类推。
- 数据两级目录：storage服务器在每个虚拟磁盘路径下创建的两级目录，用于存储数据文件。
- 文件名：与文件上传时不同。是由存储服务器根据特定信息生成，文件名包含：源存储服务器IP地址、文件创建时间戳、文件大小、随机数和文件拓展名等信息。

## 文件同步

> 写文件时，客户端将文件写至group内一个storage server即认为写文件成功，storage server写完文件后，会由后台线程将文件同步至同group内其他的storage server。
>
> 每个storage写文件后，同时会写一份binlog，binlog里不包含文件数据，只包含文件名等元信息，这份binlog用于后台同步，storage会记录向group内其他storage同步的进度，以便重启后能接上次的进度继续同步；进度以时间戳的方式进行记录，所以最好能保证集群内所有server的时钟保持同步。
>
> storage的同步进度会作为元数据的一部分汇报到tracker上，tracke在选择读storage的时候会以同步进度作为参考。

## 下载过程

>  客户端uploadfile成功后，会拿到一个storage生成的文件名，接下来客户端根据这个文件名即可访问到该文件。 

![1632566876032](../images/1632566876032.png)

跟upload file一样，在downloadfile时客户端可以选择任意tracker server。
tracker发送download请求给某个tracker，必须带上文件名信息，tracke从文件名中解析出文件的group、大小、创建时间等信息，然后为该请求选择一个storage用来服务读请求。
tracker根据请求的文件路径即文件ID 来快速定义文件。

> 如果是基于Web的http请求，此处的client可以是Nginx代理服务。下面这张图更加形象的描述了相关的流程

![1632568103583](../images/1632568103583.png)
