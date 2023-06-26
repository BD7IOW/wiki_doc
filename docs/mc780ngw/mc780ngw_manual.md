


> mc780ngw使用说明

## 概述

![产品主图](/image/main.png)

mc780ngw是基于合宙([Luat社区 (openluat.com)](https://doc.openluat.com/wiki/45?wiki_page_id=4795))Air780EX模组开发的带**RS485/RS232接口/0-24ma模拟量**数据采集的RTU/数据集中器产品。
### 应用领域
- 农业自动化/物联网数据集中器
- 自动化测试
- 环境监测

## 参数

### 4G LTE参数：
> 模组参数引用自合宙官网，这里不在实际中测试。
- __温度范围：__
   核心模组正常工作温度：-35°C ～ +70°C
    核心模组极限工作温度：-40°C ～ +85°C
- __LTE特性：__
  最大支持non-CA CAT1
  支持1.4 ～ 20MHz射频宽带
__LTE-FDD__ 最大上行速率5Mbps，最大下行速率10Mbps
__LTE-TDD：__
__上下行配置1 ：__
最大上行速率4Mbps，最大下行速率6Mbps  
__上下行配置2 ：__
最大上行速率2Mbps，最大下行速率8Mbps
- __支持频段：__
LTE-FDD：B1/B3/B5/B8
LTE-TDD：B34/B38/B39/B40/B41
- __发射功率：__
 LTE-FDD：Class3（23dBm±2dB）
LTE-TDD：Class3（23dBm+1/-3dB）
- __开发方式：__
  AT指令/二次开发（Lua /CSDK）
### ADC精度

mc780ngw（Air780EX模组）的ADC精度12bit，系统读取adc值直接返回mV数据，在Demo2中有对输入电流ADC功能进行测试，30℃室温三次平均下还是有最大+-0.2mA的跳动。
### 接口速率
RS485和RS232接口单个一对一测试最高支持115200波特率。另外485接口外挂设备个数/通信距离和通信速率，接口电容，接口上拉电阻等等关系很大。mc780ngw 485接口AB线上下拉电阻47k，120匹配电阻没有焊接。如需外挂较多设备或者通信距离很长的应用场景可能需要仔细调整程序参数或者硬件配置。
### 关于功耗
mc780ngw基于合宙Air780EX，外围器件不多，功耗数据请参考Air780EX文档。
## 功能
- [x] 支持自由通信RS232接口（1个）。
- [x] 支持自由通信RS485接口（1个）。
- [x] 0-24ma输入接口（1个）。
- [x] 支持AT 4G Cat1 modem（默认AT固件通过232接口以AT指令方式进行4G通信）。
- [x] 支持基于Lua二次开发（Luatos固件，采用Lua脚本开发）。

## 使用方法
- 以下步骤为mc780ngw典型开发步骤：
  1. 打开外壳插入SIM卡（SIM卡建议先用正常的4G手机卡测试，待脚本开发完成后再插物联网卡测试）；
  2. 连接4G天线；
  3. 如有外部设备（如485 modbus RTU设备）请按接口定义连接；
  4. 设备上电（调试时也可以直接USB供电）
  5. Luatools.exe工具看是否检测到设备串口（一般插上设备后会检测到多个串口）；
  6. 用VSCode开发脚本；
  7. 用luatools.exe下载脚本（一般选免boot下载）；
- 注意：
  如果固件下载/脚本下载失败，或者提示先按boot按键那按boot键不放再按旁边的reset键放开reset键后luatools软件提示可以放就可以放开boot键了。

## 注意事项

- **不适用于生命安全领域**：mc780ngw不能应用于生命安全领域，例如医疗设备、飞行器等。这是因为mc780ngw的通信可靠性和实时性无法满足这些领域的要求，可能会导致严重的后果。
- **不适用于可靠性要求很高的领域**：mc780ngw也不适用于可靠性要求很高的领域，例如核电站、航天器等。这是因为mc780ngw的通信可靠性和稳定性无法满足这些领域的要求，可能会导致严重的后果。
- **注意数据安全**：mc780ngw在通信过程中可能会涉及到敏感数据，例如个人隐私、商业机密等。因此，在使用mc780ngw设备时，需要注意数据的安全性，采取相应的安全措施，例如加密传输、访问控制等。
- **注意设备安全**：mc780ngw在使用过程中也需要注意设备的安全性，例如防止设备被盗、防止设备被篡改等。可以采取相应的安全措施，例如设备锁定、设备监控，脚本加密等。
- **遵守相关法律法规**：在使用mc780ngw设备时，需要遵守相关的法律法规，例如网络安全法、数据保护法等。同时，也需要遵守运营商的相关规定，例如不得恶意攻击网络、不得干扰其他用户等。

## 常见问题解答

1. 直接USB供电/5V供电/24V供电时，板子电源部分会发异响不影响正常使用。
2. 开发环境建议win10以上。
3. 有时候USB插入后设备管理串口那看不到模组的正常调试口，这时需要结合板子boot按键和reset按键下载程序，按boot键也无法下载脚本，但按boot+复位键，luatools提示发现串口，则可以结合boot键重新刷AT固件，之后再刷回luatos固件下载脚本。__boot和复位按键都看不到调试串口，这时换一条串口线试试或者换个usb口，我这里试过电脑只有一个USB口接一个扩展坞，从扩展坞通过一条长的usb线连接设备，有时就会出现上述情况，把usb线直接接电脑口就可以看到调试口出现了。__

## 应用案例
提供一些使用产品的例子，目前数据上云的例子已经在实际的环境中使用了。
两个案例的源码下载连接请看文末的网盘连接。

### demo1 电力仪表数据上云

早期有些modbus应用的电力仪表一般是配合本地PLC使用，现在数据化改造需要将这部分带通信功能设备上云，这里我们需要将一台电力仪表的数据通过mqtt发送到云平台上进行处理，通信协议采用Json字符串。mc780ngw只往上发数据不需要处理平台返回的指令。
该型号电力仪表modbus rtu数据命令数据格式：
![HED-D5电力仪表寄存器定义](/image/HED-D5_reg.png)

modbus rtu slave模拟：
![输入图片说明](/image/modbus_slave.png)
数据定义为Float AB CD,下面演示的是数据采集，解析完成后通过json发送到mqtt服务器的脚本。这个脚本只是实现简单的采集上报功能，实际使用需要更多考虑可靠性以及可维护性。比如这个项目需要部署到异地，工程师不可能大老远跑到现场就为了升级脚本，所以需要远程脚本升级功能（([Luat社区-Air780EX官方FOTA说明(openluat.com)](https://doc.openluat.com/wiki/40?wiki_page_id=4632#_iotcorescript_23))）。

```lua
-- lua代码 采集modbus数据，上传mqtt服务器
PROJECT = "modbus_rtu"
VERSION = "1.0.0"
sys = require("sys")
--[[特别注意, 使用mqtt库需要下列语句]]
sysplus = require("sysplus")
PRODUCT_KEY = "ZrQPIS6gjJhfRfWhzD9MG8PvoMP0pBAC"--这是我的key，使用是换自己的key
libnet = require "libnet"--下载的时候记得添加这个库
libfota = require "libfota""--下载的时候记得添加这个库
--根据自己的服务器修改以下参数
local mqtt_host = "lbsmqtt.airm2m.com"
local mqtt_port = 1884
local mqtt_isssl = false
local client_id = "abc"
local user_name = "user"
local password = "password"
local pub_topic = "/luatos/pub/" .. (mcu.unique_id():toHex())
--local sub_topic = "/luatos/sub/" .. (mcu.unique_id():toHex())
local mqttc = nil
--存放数据的table，后面用json库直接转为string
--[[注意table参数不能为空]]
local mqt_data = {type="upload",dev_0={"0.0.0.0"},dev_1={
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0}}
--[[上传到mqt服务的json数据格式
{"type" : "upload","dev_0" : [ "192.168.199.106" ],
"dev_1" : [ 33200.61, 21.78, 1929.6, 5327.24, 50.0, 0.5, 223.4, 10.0, 2.0, 3.0, 0.6, 222.6, 4.0, 5.0, 6.0, 7.0, 223.3, 8.0, 9.0, 1.0, 0.5 ]}
]]
local UART_BAUD = 9600--modbus rtu波特率
local UART_ID = 2--modbus rtu使用的串口号
local UART_BUFF_LEN = 2048
-- 添加硬狗防止程序卡死
if wdt then
   wdt.init(9000) -- 初始化watchdog设置为9s
   log.info("wdt","wdt init")
   sys.timerLoopStart(wdt.feed, 3000) -- 3s喂一次狗
end
uart.on(UART_ID,"receive",function(id,len)
   sys.publish("UART_RECEIVE")
   log.info("uart", "receive", id, len)
end)
--回调函数声明需要放到初始化前面，放到后面回调函数好像是不响应的
uart.on(UART_ID, "sent", function(id)
log.info("uart", "sent", id)
end)
-- 485自动切换, 选取GPIO29作为收发转换脚
gpio.setup(29, 0, gpio.PULLUP)-- 将PB3设置为输出模式
--[[
uart.setup(id, baud_rate, data_bits, stop_bits, partiy, bit_order, buff_size, rs485_gpio, rs485_level, rs485_delay)
int串口id, uart0写0, uart1写1, 如此类推, 最大值取决于设备
int波特率, 默认115200，可选择波特率表:{2000000,921600,460800,230400,115200,57600,38400,19200,9600,4800,2400}
int数据位，默认为8, 可选 7/8
int停止位，默认为1, 根据实际情况，可以有0.5/1/1.5/2等
int校验位，可选 uart.None/uart.Even/uart.Odd
int大小端，默认小端 uart.LSB, 可选 uart.MSB
int缓冲区大小，默认值1024
int485模式下的转换GPIO, 默认值0xffffffff
int485模式下的rx方向GPIO的电平, 默认值0
int485模式下tx向rx转换的延迟时间，默认值12bit的时间，单位us
波特率不同这个延时时间需要相应调整，否则会出现数据发送不完整的现象
]]
uart.setup(UART_ID, UART_BAUD, 8, 1, uart.NONE, uart.LSB, UART_BUFF_LEN, 29, 0, 3000)
--[[
-- 起始 地址 功能代码 数据 CRC校验 结束c
-- 3.5 字符 8 位 8 位 N x 8 位 16 位 3.5 字符
--- 发送modbus数据函数
@function modbus_send
@param slaveaddr : 从站地址
Instructions:功能码
reg : 寄存器编号
value : 写入寄存器值或读取寄存器个数,2字节
@return 无
@usage modbus_send("0x01","0x01","0x0101","0x04")
]]
local function modbus_send(slaveaddr,Instructions,reg,value)
  local data = (string.format("%02x",slaveaddr)..string.format("%02x",Instructions)..str ing.format("%04x",reg)..string.format("%04x",value)):fromHex()
  local modbus_crc_data= pack.pack('<h', crypto.crc16("MODBUS",data))
  local data_tx = data..modbus_crc_data
  uart.write(UART_ID,data_tx)
end
local function pub_data(pub_topic, data,qos)
  if mqttc and mqttc:ready() then
      local pkgid = mqttc:publish(pub_topic, data, qos)
  end
end
-- 返回字符串tonumber的转义字符串(用来支持超过31位整数的转换)
-- @string str 输入字符串
-- @return str 转换后的lua 二进制字符串
-- @return len 转换了多少个字符
-- @usage
-- string.toValue("123456") -> "\1\2\3\4\5\6" 6
-- string.toValue("123abc") -> "\1\2\3\a\b\c" 6
--function string.toValue(str)
-- return string.fromHex(str:gsub("%x", "0%1"))
-- end
local function modbus_read()
   local cacheData = ""
   while true do
      local s = uart.read(UART_ID,1)
      if s == "" then
        --数据太长会分包，注意处理
         if not sys.waitUntil("UART_RECEIVE",20) then
             if cacheData:len()>0 then
                local rev_crc = 0
                local nexti, num_byte = pack.unpack(cacheData, "b",3)
                if num_byte ~=nil and cacheData:len() == 5+num_byte then
                   local a,_ = string.toHex(cacheData)
                   log.info("modbus 接收字节:",a)
                   nexti, rev_crc = pack.unpack(cacheData, "<H",nexti+num_byte)
                   log.info("modbus接收crc:",string.format("%02x",rev_crc))
                   local calcrc = crypto.crc16("MODBUS",string.sub(cacheData, 1, 3+num_byte))
                   if rev_crc == calcrc then
                      log.info("modbus CRC正确")
                      nexti,mqt_data.dev_1[1] = pack.unpack(cacheData, ">f",4)--从第4个字节开始
                      for i=2,21 do
                          nexti,mqt_data.dev_1[i]=pack.unpack(cacheData, ">f",nexti)--迭代解析后续数据
                      end
                      log.info("json",json.encode(mqt_data))
                      pub_data(pub_topic,json.encode(mqt_data),0)
                   end
                   cacheData=""
                else
                   if cacheData:len() > (UART_BUFF_LEN/2) then
                         log.info("data too much")
                         cacheData=""
                   end
               end
          end
       end
    else
       cacheData = cacheData..s
    end
  end
end
sys.taskInit(modbus_read)
sys.taskInit(function ()
      while true do
            sys.wait(10000)
          --21个数据每个4位，一共84字节，按照modbus协议数据以2字节为一个数据所有需要读取42=0x2A
            modbus_send("0x01","0x04","0x0000","0x2A")
      end
end)

sys.taskInit(function()

--local device_id = mcu.unique_id():toHex()
--LED = gpio.setup(27, 0, gpio.PULLUP)
    device_id = mobile.imei()
    -- 默认都等到联网成功
    sys.waitUntil("IP_READY")
    sys.publish("net_ready", device_id)
end)
sys.taskInit(function()
-- 等待联网
    local ret, device_id = sys.waitUntil("net_ready")
-- 下面的是mqtt的参数均可自行修改
    client_id = device_id
    pub_topic = "/mc780ngw/pub/" .. device_id
-- 打印一下上报(pub)和下发(sub)的topic名称
-- 可使用mqtt.x等客户端进行调
    log.info("mqtt", "pub", pub_topic)
--mqtt.create(adapter,host,port,ssl,isipv6)
    mqttc = mqtt.create(nil, mqtt_host, mqtt_port, mqtt_isssl, ca_file)
    mqttc:auth(client_id,user_name,password) -- client_id必填,其余选填
    -- mqttc:keepalive(240) -- 默认值240s
    mqttc:autoreconn(true, 3000) -- 自动重连机制
    mqttc:on(function(mqtt_client, event, data, payload)
    log.info("mqtt", "event", event, mqtt_client, data, payload)
    if event == "conack" then
    -- 联上了
       sys.publish("mqtt_conack")
    elseif event == "sent" then
       log.info("mqtt", "sent", "pkgid", data)
    --elseif event == "disconnect" then
    -- 非自动重连时,按需重启mqttc
    -- mqtt_client:connect()
    end
end)
-- mqttc自动处理重连, 除非自行关闭
mqttc:connect()
sys.waitUntil("mqtt_conack")
while true do
-- 等待其他task发送过来的上报信息
local ret, topic, data, qos = sys.waitUntil("mqtt_pub", 300000)
if ret then
-- 提供关闭本while循环的途径, 不需要可以注释掉
   if topic == "close" then break end
   mqttc:publish(topic, data, qos)
end
end
mqttc:close()
mqttc = nil
end)
-- 用户代码已结束---------------------------------------------
-- 结尾总是这一句
sys.run()
-- sys.run()之后后面不要加任何语句!!!!!
```

上面代码的效果，可见已经成功把数据上传到mqt服务器了：
![mqt消息订阅](/image/mqt-rev.png)
FOTA远程升级涉及的代码如下：
```lua
PRODUCT_KEY = "ZrQPIS6gjJhfRfWhzD9MG8PvoMP0pBAC"--换自己的key
libnet = require "libnet"
libfota = require "libfota"
function fota_cb(ret)
   log.info("fota", ret)
   if ret == 0 then
      rtos.reboot() -- FOTA完成, 重启
   end
end
-- 使用合宙iot平台进行升级
libfota.request(fota_cb)
-- 这里写10分钟是为了更快OTA到下一个版本, 请按实际情况判断
sys.timerLoopStart(libfota.request, 600000, fota_cb)
```
每上电检查一次是否升级，之后每10分钟检测是否需要升级。
运行程序后[合宙IOT](https://iot.openluat.com/)平台（需要自己注册一个账户）上能看到这个设备：

![合宙IOT后台OTA](/image/ota1.png)
这里之后就可以通过luattools工具生成量产文件上传到合宙iot后台远程升级了。

### demo2 综合测试
- 这个例子对mc780ngw 4G联网功能、485接口、232接口、0-24ma输入接口、LED状态灯测试。
- 脚本程序实现的功能：mc780ngw开机SNTP获取网络时间、获取基站定位，通过基站定位得到经纬度然后利用[心知天气](https://www.seniverse.com/)获取设备位置的天气状态，并显示在HMI串口屏上。同时mc780ngw定时通过modbus 485采集温湿度数据、通过模拟量接口获取0-24ma输入电流数据一并通过232接口显示在HMI串口屏上，最后mc780ngw将所有数据通过Mqtt上传到[OneNET - 中国移动物联网开放平台](https://open.iot.10086.cn/)上。hmi显示屏有一个按键，按键按下后发送通知到mc780ngw，随后mc780ngw控制显示屏播放小地球gif图片，按关闭，小地球停止播放。

```lua
PROJECT = "modbus_rtu"
VERSION = "1.0.2"
sys = require("sys")
--[[特别注意, 使用http库需要下列语句]]
sysplus = require("sysplus")
lbsLoc = require("lbsLoc")
libnet = require ("libnet")
--全局变量,注意变量的声明最好放在引用之前，否则可能会出现意想不到的行为
--*********************************
local UART_BAUD = 115200--modbus rtu波特率
local UART_ID = 2--modbus rtu使用的串口号
local DEV_ADDR = 1--温湿度传感器模块地址
local HMI_UART_BAUD = 115200
local HMI_UART_ID = 1
local temp = 1--温度值，原始值/10
local humd = 1--湿度值，原始值/10
local cacheData = ""--modbus采集缓存
local INPUT_VOL_PIN = 0--外部电压检测adc引脚
local INPUT_CUR_PIN = 1--0-24ma电流检测adc引脚
local location = "21.9270:110.8943" --经纬度定位 维度:经度
local priv_key = "STEsQuQke6gIKfI0N"--注册心知天气 获取免费的私钥
local req_url = ""--心知天气 数据请求的url 需要注册 用的是密钥最简url，没使用验证签名
PRODUCT_KEY = "ZrQPIS6gjJhfRfWhzD9MG8PvoMP0pBAC"--量产项目中一定要使用自己在iot.openluat.com中创建的项目productKey
local isBtnOpen=false --模拟一个开关，对应hmi显示屏上按键打开后小地球gif开始动作，否则静态不动
local wth_text=""--存放天气状态
local date_str = ""--存放日期
local time_str = ""--存放当前时间
local Screen_id = 0
local hmi_date_id=7
local hmi_time_id = 8
local hmi_sensor_temp_id = 6
local hmi_sensor_humd_id = 14
local hmi_sensor_cur_id = 17
local hmi_location_id = 12
local hmi_ama_id = 4
local hmi_btn_id = 2
--[[
本demo演示的是 OneNet Studio, 注意区分
https://open.iot.10086.cn/studio/summary
https://open.iot.10086.cn/doc/v5/develop/detail/iot_platform
]]
-- 根据自己的设备修改以下参数
----------------------------------------------
-- OneNet Studio
mqtt_host = "mqtts.heclouds.com"
mqtt_port = 1883
mqtt_isssl = false
local pid = "2tdgjKHle5" -- 产品id,注意换成自己的
local device = "mc780gw" -- 设备名称, 按需设置, 如果是Cat.1系列通常用mobile.imei(),注意换成自己的,平台上设置和这里一样
local device_secret = "TFNzYmMzV3VmNFhRNXFCYnppREhObEhsWGlMM3hGRW4=" -- 设备密钥,注意换成自己的
client_id, user_name, password = iotauth.onenet(pid, device, device_secret)
-- 下面是常用的topic, 完整topic可参考 https://open.iot.10086.cn/doc/v5/develop/detail/639
pub_topic = "$sys/" .. pid .. "/" .. device .. "/thing/property/post"
sub_topic = "$sys/" .. pid .. "/" .. device .. "/thing/property/set"
pub_custome = "$sys/" .. pid .. "/" .. device .. "/custome/up"
pub_custome_reply = "$sys/" .. pid .. "/" .. device .. "/thing/property/post/reply"
sub_custome = "$sys/" .. pid .. "/" .. device .. "/custome/down/+"
sub_custome_reply = "$sys/" .. pid .. "/" .. device .. "/custome/down_reply/"
------------------------------------------------
local mqttc = nil
local payloads = {}
local PowerVol = 0
local SensorCur =0--需要校准要不然电流检测的精度还是不好的
--local res_val=50.9--检测电阻
local res_avg_num=0
local res_avg_tb={}--多次平均
--local cur_input=0
--*********************************
--初始化
--LED引脚判断赋值结束
--local NET_LED= gpio.setup(27, 0, gpio.PULLUP)
-- 添加硬狗防止程序卡死
--if wdt then
-- wdt.init(9000) -- 初始化watchdog设置为9s
-- log.info("wdt","wdt init")
-- sys.timerLoopStart(wdt.feed, 3000) -- 3s喂一次狗
--end
adc.open(INPUT_VOL_PIN)--电源电压检测ADC
adc.open(INPUT_CUR_PIN)--输入电流检测ADC
-- 功能:获取基站对应的经纬度后的回调函数
-- 参数:-- result：number类型，0表示成功，1表示网络环境尚未就绪，2表示连接服务器失败，3表示发送数据失败，4表示接收服务器应答超时，5表示服务器返回查询失败；为0时，后面的5个参数才有意义
-- lat：string类型，纬度，整数部分3位，小数部分7位，例如031.2425864
-- lng：string类型，经度，整数部分3位，小数部分7位，例如121.4736522
-- addr：目前无意义
-- time：string类型或者nil，服务器返回的时间，6个字节，年月日时分秒，需要转为十六进制读取
-- 第一个字节：年减去2000，例如2017年，则为0x11
-- 第二个字节：月，例如7月则为0x07，12月则为0x0C
-- 第三个字节：日，例如11日则为0x0B
-- 第四个字节：时，例如18时则为0x12
-- 第五个字节：分，例如59分则为0x3B
-- 第六个字节：秒，例如48秒则为0x30
-- locType：numble类型或者nil，定位类型，0表示基站定位成功，255表示WIFI定位成功
local function getLocCb(result, lat, lng, addr, time, locType)
   log.info("testLbsLoc.getLocCb", result, lat, lng)
   -- 获取经纬度成功
  if result == 0 then
     location=string.sub(lat,2,-1)..":"..lng
     log.info("服务器返回的经纬度", location)
     log.info("服务器返回的时间", time:toHex())
     log.info("定位类型,基站定位成功返回0", locType)
   end
    -- 广播给其他需要定位数据的task
    sys.publish("lbsloc_result", result, lat, lng)
end
--[[
大彩串口屏显示
合宙系列模块Lutos其实是原本支持TFT液晶显示的，但是mc780gw没有把相关接口引出来
mc780gw主打的是可靠485，232通用敏捷开发小网关/数据集中器，没法做到将官方的每个接口和功能都兼顾。
大彩hmi串口屏有自己的C语言驱动库，这里也没有做相关lua移植，作为demo只做其部分指令的简单适配。
有兴趣的大神可以造一个轮子^-^
hmi屏指令参考官网文档<<大彩串口屏指令集V5.1.PDF>>
]]
uart.setup(HMI_UART_ID, HMI_UART_BAUD, 8, 1, uart.NONE)
uart.on(HMI_UART_ID,"receive",function(id,len) --uart.on如放到uart.setup前面似乎不行，on不起作用了
     local s = ""
-- 如果是air302, len不可信, 传1024
-- s = uart.read(id, 1024)
--串口解析的可靠性还需要多些考虑，防止非法字符导致系统崩溃
     s = uart.read(id, len)
     if #s > 0 then -- #s 是取字符串的长度
-- 如果传输二进制/十六进制数据, 部分字符不可见, 不代表没收到
-- 关于收发hex值,请查阅 https://doc.openluat.com/article/583
         log.info("hmi", "receive", id, #s, s:toHex())
         local _,head = pack.unpack(s, "b",1)
         local _,tail = pack.unpack(s, "<I",#s-3)
         log.info("hmi", "ht", head, tail)
         if head==0xEE and tail==0xFFFFFCFF then
--EE B1 11 00 00 00 02 10 01 01 FF FC FF FF
--EE B1 11 00 00 00 02 10 01 00 FF FC FF FF
            local _,sc_id = pack.unpack(s, ">h",4)--屏幕id
            local _,ctl_id =pack.unpack(s, ">h",6)--控件id
            log.info("hmi", "sc", sc_id, ctl_id)--Screen_id hmi_btn_id
            if sc_id == Screen_id and ctl_id == hmi_btn_id then
-- body
               local _,btn_val = pack.unpack(s, "b",10)--按键值
               log.info("hmi", "bt_val", btn_val)
               local btn_data = ""
               if btn_val >0 then
                 isBtnOpen = true
 btn_data="EEB11000"..string.format("%02x",Screen_id).."00"..string.format("%02x",hmi_btn_id).."01".."FFFCFFFF"
--hmi_ama_id
btn_data=btn_data.."EEB12000"..string.format("%02x",Screen_id).."00"..string.format("%02x",hmi_ama_id).."FFFCFFFF"
              else
                isBtnOpen=false
btn_data="EEB11000"..string.format("%02x",Screen_id).."00"..string.format("%02x",hmi_btn_id).."00".."FFFCFFFF"
btn_data=btn_data.."EEB12100"..string.format("%02x",Screen_id).."00"..string.format("%02x",hmi_ama_id).."FFFCFFFF"
              end
              log.debug("突发消息准备发送",btn_data)
           sys.publish("hmi_update_now",btn_data)--通知更新小地球的值和按键状态
         else

         log.debug("sc_id or btn_id error","....")

         end
   end
-- log.info("hmi", "receive", id, #s, s:toHex())
-- local ind,tt = pack.unpack(s, "<I",#s-3)
-- log.info("hmi", "tt=",tt )
-- log.info("hmi", "index=",ind )
-- log.info("hmi", "ttf", string.format("0x%04X",tt))
--uart.write(id, s)
   end
end)
uart.on(HMI_UART_ID, "sent", function(id)
log.info("hmi", "sent", id)
end)
--[[
uart.setup(id, baud_rate, data_bits, stop_bits, partiy, bit_order, buff_size, r485_gpio, rs485_level, rs485_delay)
int串口id, uart0写0, uart1写1, 如此类推, 最大值取决于设备
int波特率, 默认115200，可选择波特率表:{2000000,921600,460800,230400,115200,57600,38400,19200,9600,4800,2400}
int数据位，默认为8, 可选 7/8
int停止位，默认为1, 根据实际情况，可以有0.5/1/1.5/2等
int校验位，可选 uart.None/uart.Even/uart.Odd
int大小端，默认小端 uart.LSB, 可选 uart.MSB
int缓冲区大小，默认值1024
int485模式下的转换GPIO, 默认值0xffffffff
int485模式下的rx方向GPIO的电平, 默认值0
int485模式下tx向rx转换的延迟时间，默认值12bit的时间，单位us
波特率不同这个延时时间需要相应调整，否则会出现数据发送不完整的现象
]]
uart.setup(UART_ID, UART_BAUD, 8, 1, uart.NONE, uart.LSB, 2048, 29, 0, 3000)
-- 485自动切换, 选取GPIO29作为收发转换脚
gpio.setup(29, 0, gpio.PULLUP)-- 设置为输出模式
uart.on(UART_ID,"receive",function(id,len)
sys.publish("UART_RECEIVE")
log.info("uart", "receive", id, len)
end)
uart.on(UART_ID, "sent", function(id)
log.info("uart", "sent", id)
end)
local function modbus_read()
  while true do
      local s = uart.read(UART_ID,1)
      if s == "" then
      if not sys.waitUntil("UART_RECEIVE",20) then
        if cacheData:len()>0 then
         local a,_ = string.toHex(cacheData)
         local nexti, num_byte=    pack.unpack(cacheData, "b",3)
--log.info("modbus 接收字节:",a)
--log.info("modbus payload字节数:",num_byte)
         local rev_crc = 0

         if num_byte ~=nil and cacheData:len() == 5+num_byte then
-- body
            nexti, rev_crc = pack.unpack(cacheData, "<H",nexti+num_byte)
--log.info("modbus接收crc:",string.format("%02x",rev_crc))
           local calcrc =     crypto.crc16("MODBUS",string.sub(cacheData, 1, 3+num_byte))
--log.info("modbus计算crc:",string.format("%02x",calcrc))
           if rev_crc == calcrc then
-- body
           log.info("modbus CRC正确")
           nexti,temp = pack.unpack(cacheData, ">h",4)--第4字节开始，>h 字节序高低，两字节
          nexti,humd = pack.unpack(cacheData, ">h",nexti)
--log.info("读取的温度值:",temp/10.0)
--log.info("读取的湿度值:",humd/10.0)
end
end
cacheData=""
end
end
else
cacheData = cacheData..s
end
end
end
--uart.on(1,"receive",function() sys.publish("RS232_RECEIVE") end)
--uart.setup(1,UART_BAUD,8,uart.PAR_NONE,uart.STOP_1)
local function   modbus_send(uart_id,slaveaddr,Instructions,reg,value)
--log.info("发送数据采集命令：", uart_id..slaveaddr..Instructions..reg..value)
local data = (string.format("%02x",slaveaddr)..string.format("%02x",Instructions)..string.format("%04x",reg)..string.format("%04x",value)):fromHex()
local modbus_crc_data= pack.pack('<h', crypto.crc16("MODBUS",data))
local data_tx = data..modbus_crc_data
uart.write(uart_id,data_tx)
end
sys.taskInit(modbus_read)
--sys.taskInit(rs232_read)
local function wth_parse(data)
-- body
--[[
{
"results": [{
"location": {
"id": "WS0E9D8WN298",
"name": "广州",
"country": "CN",
"path": "广州,广州,广东,中国",
"timezone": "Asia/Shanghai",
"timezone_offset": "+08:00"
},
"now": {
"text": "晴",
"code": "0",
"temperature": "32"
},
"last_update": "2023-05-31T08:50:15+08:00"
}]
]]
local tjsondata, result, errinfo = json.decode(data)
if result and type(tjsondata) == "table" then
if tjsondata["results"]==nil then
wth_text= "无效".." ".."无效".." ".."0"
log.error("json err","no results key")
return
end
local results = tjsondata["results"][1]
--if results~=nil then
-- body
local name = results["location"]["name"]
if name==nil then
log.info("testJson.decode error", errinfo)
wth_text= "无效".." ".."无效".." ".."0"
return
end
log.info("name:", name)
local text = results["now"]["text"]
local temperature = results["now"]["temperature"]
log.info("text:", text)
log.info("temp", temperature)
wth_text = name.." "..text.." "..temperature
--else
-- log.info("testJson.decode error", errinfo)
-- wth_text= "无效".." ".."无效".." ".."0"
--end
else
log.info("testJson.decode error", errinfo)
wth_text= "无效".." ".."无效".." ".."0"
end
end
gpio.setup(27, 0, gpio.PULLUP)-- net led设置为输出模式
-- 断网后会发一次这个消息
sys.subscribe("IP_LOSE", function(adapter)
log.info("w5500", "IP_LOSE", (adapter or -1) == socket.ETH0)
gpio.set(27,0)
end)
sys.taskInit(function()
--注册串口的数据发送通知函数
-- uart.on(UART_ID,"receive",read_test)
device_id = mobile.imei()
--log.info("ipv6", mobile.ipv6(true))
log.info("device_id(imei):",device_id)
sys.waitUntil("IP_READY", 30000)
sys.publish("net_ready", device_id)
lbsLoc.request(getLocCb)--获取经纬度参数
local res,Locres, Loclat, Loclng= sys.waitUntil("lbsloc_result",20000)--基站定位不会总是成功的，有些地方很容易，有些不行，就比如我家就老定位失败，公司每次都成功
if res==false then
location="21.9270:110.8943"
log.info("获取基站定位失败，将使用默认定位数据", location)
end
req_url="https://api.seniverse.com/v3/weather/now.json?key="..priv_key.."&location="..location.."&language=zh-Hans&unit=c"
log.info("req_url",req_url)
local code, headers, body = http.request("GET", req_url).wait()
log.info("http.get", code, headers, body)
wth_parse(body)
local on_time_update_wth = 1
res_avg_num=1
while true do
--gpio.set(29, gpio.HIGH) -- 输出高电平
modbus_send(UART_ID,DEV_ADDR,"0x04","0x01","0x02")
--modbus_send(1,DEV_ADDR,"0x04","0x01","0x02")
on_time_update_wth=on_time_update_wth+1
if on_time_update_wth > 1800 then
--每间隔1小时更新一次天气数据
code, headers, body = http.request("GET", req_url).wait()
log.info("http.get", code, headers, body)
wth_parse(body)
on_time_update_wth=0
end
sys.wait(2000)
PowerVol = adc.get(INPUT_VOL_PIN)
res_avg_tb[res_avg_num]=adc.get(INPUT_CUR_PIN)
log.debug("adc", "adc" .. tostring(INPUT_VOL_PIN), PowerVol) -- 若adc.get报nl, 改成adc.read
log.debug("adc", "adc" .. tostring(INPUT_CUR_PIN), res_avg_tb[res_avg_num]) -- 若adc.get报nil, 改成adc.read
res_avg_num=res_avg_num+1
if res_avg_num >= 4 then
res_avg_num=1
local cur_sum = (res_avg_tb[1]+res_avg_tb[2]+res_avg_tb[3])/3
SensorCur=cur_sum*0.0201-0.1
end
end
end)
local function on_downlink( top,pay )
-- body
log.info("down top:",top)
log.info("down payload:",pay)
end
sys.taskInit(function()-- 等待联网
sys.waitUntil("net_ready")
if mqtt == nil then
while 1 do
sys.wait(3000)
log.info("bsp", "本固件未包含mqtt库, 请查证")
end
end
gpio.set(27,1)--联网成功
-- 打印一下上报(pub)和下发(sub)的topic名称
-- 上报: 设备 ---> 服务器
-- 下发: 设备 <--- 服务器
-- 可使用mqtt.x等客户端进行调试
log.info("mqtt", "pub", pub_topic)
log.info("mqtt", "sub", sub_topic)
log.info("mqtt", mqtt_host, mqtt_port, client_id, user_name, password)
mqttc = mqtt.create(nil, mqtt_host, mqtt_port, mqtt_isssl, ca_file)
mqttc:auth(client_id, user_name, password) -- client_id必填,其余选填
-- mqttc:keepalive(240) -- 默认值240s
mqttc:autoreconn(true, 3000) -- 自动重连机制
mqttc:on(function(mqtt_client, event, data, payload)
-- 用户自定义代码
log.info("mqtt", "event", event, mqtt_client, data, payload)
if event == "conack" then
-- 联上了
sys.publish("mqtt_conack")
local topics = {}
-- 物模型的topic
topics[sub_topic] = 2
-- 透传模式的topic
-- 首先是 上报后, 服务器会回复
if pub_custome_reply then
topics[pub_custome_reply] = 1
end
-- 然后是 服务器的下发
if sub_custome then
topics[sub_custome] = 1
end
-- mqtt_client:subscribe(sub_topic, 2)--单主题订阅
mqtt_client:subscribe(topics) -- 多主题订阅
elseif event == "recv" then
-- 打印收到的内容, 时间生产环境建议注释掉, 不然挺多的
log.info("mqtt", "downlink", "topic", data, "payload", payload)
on_downlink(topic, payload)
elseif event == "sent" then
log.info("mqtt", "sent", "pkgid", data)
-- elseif event == "disconnect" then
-- 非自动重连时,按需重启mqttc
-- mqtt_client:connect()
end
end)
-- mqttc自动处理重连, 除非自行关闭

mqttc:connect()
sys.waitUntil("mqtt_conack")
while true do
-- 演示等待其他task发送过来的上报信息
local ret, topic, data, qos = sys.waitUntil("mqtt_pub", 300000)
if ret then
-- 提供关闭本while循环的途径, 不需要可以注释掉
if topic == "close" then
break
end
mqttc:publish(topic, data, qos)
end
-- 如果没有其他task上报, 可以写个空等待
-- sys.wait(60000000)
end
mqttc:close()
mqttc = nil
end)

sys.taskInit(function()
local qos = 0 -- QOS0不带puback, QOS1是带puback的
while true do
sys.wait(15000)
log.info("准备发布数据", mqttc and mqttc:ready())
--local s = os.time()
--.info("printTime", string.format("%04d-%02d-%02d %02d:%02d:%02d", t.year,t.month,t.day,t.hour,t.min,t.sec))
-- onenet 使用的是OneJson, 就是规范化的Json结构
-- https://open.iot.10086.cn/doc/v5/develop/detail/508
if mqttc and mqttc:ready() then
local data = {}
data["id"] = tostring(mcu.ticks())
data["params"] = {}
-- 业务自定义数据
-- 例如:
-- 温度
data["params"]["TempValue"] = {
value = temp/10.0

}
data["params"]["HumdValue"] = {
value = humd/10.0
}
data["params"]["PowerVol"] = {
value = (PowerVol/0.04761905)/1000.0--显示电源电压
}
-- PowerVol = adc.get(INPUT_VOL_PIN)
-- SensorCur = adc.get(INPUT_CUR_PIN)
data["params"]["SensorCur"] = {
value = SensorCur
}
-- 上传一个字符串
if isBtnOpen then
data["params"]["BtnValue"] = {
value = "打开"
}
else
data["params"]["BtnValue"] = {
value = "关闭"
}
end
data["params"]["wth"] = {
value = wth_text
}

local updata = json.encode(data)
log.info("mqtt", "待上报数据", updata)
local pkgid = mqttc:publish(pub_topic, updata, qos)
end
end
end)
--- utf8编码 转化为 gb2312编码
-- @string utf8s utf8编码数据
-- @return string data,gb2312编码数据
-- @usage local data = common.utf8ToGb2312(utf8s)
function utf8ToGb2312(utf8s)
local cd = iconv.open("ucs2", "utf8")
local ucs2s = cd:iconv(utf8s)
cd = iconv.open("gb2312", "ucs2")
return cd:iconv(ucs2s)
end
local function hmi_update()
--指令上传格式：EE 【B1 12 Screen_id+Control_id0+Len0+ Strings0+ +…Control_idn+Lenn+ Stringsn】FF FC FF FF
local hdata = "EEB11200"--批量更新的消息头
--local data_trail={0xFF,0xFC,0xFF,0xFF}--消息尾
hdata = hdata..string.format("%02x",Screen_id)
hdata = hdata.."00"..string.format("%02x",hmi_date_id)
hdata = hdata.."00"..string.format("%02x",#date_str)
hdata = hdata..string.toHex(date_str)
hdata = hdata.."00"..string.format("%02x",hmi_time_id)
hdata = hdata.."00"..string.format("%02x",#time_str)
hdata = hdata..string.toHex(time_str)
hdata = hdata.."00"..string.format("%02x",hmi_sensor_temp_id)
local temp_str = string.format("%.1f",temp/10.0)
hdata = hdata.."00"..string.format("%02x",#temp_str)
hdata = hdata..string.toHex(temp_str)
hdata = hdata.."00"..string.format("%02x",hmi_sensor_humd_id)
temp_str = string.format("%.1f",humd/10.0)
hdata = hdata.."00"..string.format("%02x",#temp_str)
hdata = hdata..string.toHex(temp_str)
hdata = hdata.."00"..string.format("%02x",hmi_sensor_cur_id)
temp_str = string.format("%.2f",SensorCur)
hdata = hdata.."00"..string.format("%02x",#temp_str)
hdata = hdata..string.toHex(temp_str)
local cb = utf8ToGb2312(wth_text)
hdata = hdata.."00"..string.format("%02x",hmi_location_id)--wth_text
hdata = hdata.."00"..string.format("%02x",#cb)
hdata = hdata..string.toHex(cb)
hdata = hdata.."FFFCFFFF"
if isBtnOpen then
-- body
--EE 【B1 10 Screen_id Control_id Status 】FF FC FF FF
hdata=hdata.."EEB11000"..string.format("%02x",Screen_id).."00"..string.format("%02x",hmi_btn_id).."01".."FFFCFFFF"
else
hdata=hdata.."EEB11000"..string.format("%02x",Screen_id).."00"..string.format("%02x",hmi_btn_id).."00".."FFFCFFFF"
end
uart.write(HMI_UART_ID,hdata:fromHex())
--log.info("hmi",hdata)
end

  
  

--定时刷新显示任务

sys.taskInit(function()
sys.waitUntil("IP_READY",30000)
sys.subscribe("NTP_UPDATE", function()
local time = os.date("*t")
log.info("os.time",time.year, time.month, time.day, time.hour, time.min, time.sec)
rtc.set({year=time.year,time.month,time.day, time.hour, time.min,time.sec})
local t = rtc.get()
log.info("rtc", json.encode(t))
end)

-- 订阅NTP错误事件
sys.subscribe("SNTP_FAILED", function()
log.error("SNTP", "failed")
end)
-- 实际应用中需要定时同步一下时间，对时间精度/同步可靠性要求很高的应用谨慎使用ntp
-- 设置NTP服务器地址和时区
socket.sntp("ntp1.aliyun.com", 8)
-- 订阅NTP更新事件
-- 查询当前时间
local hmi_ret=fasle
local hmidata = ""
while true do
hmi_ret, hmidata = sys.waitUntil("hmi_update_now", 970)
time = os.date("*t")
date_str = string.format("%d",time.year).."-"..string.format("%d",time.month).."-"..string.format("%d",time.day)
time_str =string.format("%d",time.hour).."-"..string.format("%d",time.min).."-"..string.format("%d",time.sec)
--因为有个时间参数需要更新，定时的时间不要超过1秒,hmi屏不要频繁刷新。
if hmi_ret then
--突发更新显示
uart.write(HMI_UART_ID,hmidata:fromHex())
log.debug("突发接收",hmidata)
else
--定时更新显示
--批量更新文本控件

hmi_update()
--gb2312str=common.ucs2ToGb2312(utf8str)
----local cb = utf8ToGb2312(utf8str)
--log.debug("tag", utf8str)
--log.debug("tag", string.toHex(cb))
end
end
end)
sys.run()
-- sys.run()之后后面不要加任何语句!!!!!
```
OneNet上的效果：

![OneNet效果](/image/OneNet.png)

视频演示：
<iframe src="//player.bilibili.com/player.html?aid=955089156&bvid=BV1uW4y1D7tN&cid=1172939703&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>


## 帮助
 - 有问题欢迎联系客服->[首页-微电工作室-淘宝网](https://micro-circuits.taobao.com/)
- Lua脚本api使用（更多应用demo例子也可以在这里找到）->[LuatOS 文档](https://wiki.luatos.com/)
- Lua官方网站->[The Programming Language Lua](http://www.lua.org/)
- Lua语法基础学习->[Lua 教程 | 菜鸟教程 ](https://www.runoob.com/lua/lua-tutorial.html)
- 产品资料[mc780ngw网盘资料(提取码: y23w)](https://pan.baidu.com/s/1UnvGdk9E_YbnGywGUwY9Kg)

