# SKD Release Notes

## ChipSet

***PHY6222/PHY6252***

---

## SDK Version

***PHY62XX_SDK_3.X.X***

---

### **Version**: PHY62XX_SDK_3.1.3

### **Change List**

### **[components]**

    driver:
        boot        : 1.Add read chip harware version function,new hw version support 48M DLL
                      2.Initial AON register after wakeup process or software reset 
        flash       : Add flash lock moudle to realize flash lock/unlock
        pwrmgr      : 1.Bugfixed AON register abnormal,use rom interface
                      2.LDO low current bit set zero when CFG_SRAM_RETENTION_LOW_CURRENT_LDO_ENABLE is not define
                      3.Reset all common peripheral interrupt priority to IRQ_PRIO_HAL after wakeup 
        fs          : 1.Add CRC verify
                      2.Supports fs ID caching, enabling fast indexing of fs read and write operations
                      3.Support fs dual backup operation, which can retrieve old data in case of current file CRC error
        spi		    : 1.Bugfixed multi-bytes transmit error
                      2.Add Data Frame Size(DFS) select function
        uart        : Bugfixed uart busy when set baudrate
        pwm         : Bugfixed pwm dead zone Settings and complementary interface Settings
        bsp_button  : Use bsp button driver(SW) instead of kscan(HW)
    mesh:           :
        EM_timer    : 1.Bugfixed op EM_stop_timer when this timer expire                      
                      2.Add sensor/time/scheduler model(client/server)                      
    profiles        : 1.Add latest phy_plus_phy profiles, aka 2.4G proprietary protocol, support smart_RF/smart_nRF with Autoack or not                      
                      2.Add encryption upgrade for slb                       

### **[example]**
    example         : Remove tasksArr and tasksCnt from XIP to SRAM
    ble_mesh        : 1.mesh_gateway: Add PB-GATT provision function
                      2.Redefine cli command buffer length
                      3.Add mesh gcc demo
                      4.mesh_lpn: Bugfixed lpn receive message before poll fsn error
    ble_peripheral  : 1.When enter dtm mode,watchdog clock is disabled
                      2.SbpSmart_nRF project uses the lastest common phy_plus_phy profiles in components//profiles
    ble_multi       : 1.Add sbm AT demo(CLI) 
                      2.Add scan_duration with random values 
                      3.Bugfixed multi schedule abnormal stop                 
	peripheral  	: Use macro definitions to distinguish adc demos,No longer mixed together
    ppsp_demo  	    : Add ppsp demo
    ota_dongle      : Add ota dongle demo 
    PhyPlusPhy      : Smart_nrf project uses the lastest common phy_plus_phy profiles in components//profiles      

### **[lib]**
    rf.lib          : 1.Bugfixed fake connection
                      2.Add txdataQLen to limit notify or write_cmd
                      3.Optimize ll malloc linkbuf 
                      4.Fix phyrsp &phyupdate issue in same conn_event 
                      5.Adjust directadv delay
                      6.Adjust slave rx window for crcok
    ble_host.lib    : Optimize the transmission of received message(patch filter)                     
    mesh_lib        : 1.Bugfixed mesh stack of ltrn/trn                     
                      2.Add mesh nodeid and networkid beacon scan procedure
                      3.Add mesh patch,support mesh node and mesh gateway

### **[misc]**
    bb_rom_sym_m0   : Update some symbol tables 
---

### **Version**: PHY62XX_SDK_3.1.2

### **Change List**

### **[components]**

    driver:
        ota_flash   : use dma interface in flash write api(default)
        flash       : 1.improve spif read/write speed
					  2.optimze flash lock api
        pwrmgr      : add rc32k clock tracking init and clear calibration flag
		gpio		: add param check in gpio driver
    mesh:           : 1.add mesh config data back-up function(read&write)
					  2.change mesh limit dir for user config	
					  3.Bugfixed cbtimer stop in margin case
					  4.Bugfixed blebrr state error in scan/adv mode

### **[example]**
    ble_mesh        : 1.add fast provision
					  2.add mesh frined/lpn project
					  3.add mesh multi connect project
    ble_peripheral  : 1.add extBlePeripheral project
                      2.add sbpMultiConn project
                      3.add gcc demo
                      4.add sbpSmart_nRF project
                      5.add sbpSmartRF project
    ble_central     : 1.add extBLECentral project
                      2.add sbcMultiConn project
    ble_multi       : 1.modify the ADV schedule logic 
                      2.add slave multi timer list 
                      3.Bugfixed some node free multiple times                    
	peripheral  	: 1.resource adc driver in adc demo(interrupt,polling,compare and adc voice)
					  2.add iic demo
					  

### **[lib]**

    rf.lib          : 1.optimized 16M tracking function
                      2.add xiprestore
                      3.support advscan concurrent
                      4.add extadv function
                      5.add secscan active scan feature
                      6.adjust adv interval no 20ms limit
                      7.use the OSAL memory heap to apply for link buffer dynamically
                      8.add smart_nrf function
    ble_host.lib    : 1.single and multiple connections use the same lib                      
    mesh_lib        : 1.Bugfixed mesh stack					  
					  2.optimze mesh relay&transmit count

### **[misc]**


---

### **Version**: PHY62XX_SDK_3.1.1

### **Change List**

### **[components]**

    driver:
        ota_flash   : update ota_flash.c/slb.c to support fast slb boot
        flash       : add flash api to get flash id
        pwrmgr      : 1.add _wdt_init in wakuep_init1
                      2.add osal_idle_task in hal_pwrmgr_init when wdt enabled
                      3.watchdog_config for wtd rest cyc config and enable wdt
                      4.feed wdt in osal_idle_task
    mesh:           : support to add proxy filter list api

### **[example]**
    ble_mesh        : bugfix blebrr queue full (bleMemAllocError status process)
    ble_peripheral  : simpleBleperipheral procedure optimize
                      update system clock to DBL 32M

### **[lib]**

    rf.lib          : 1.optimize sleep wakeup procedure,add wakeupinit tracking for 32M DBL 
                      2.fix large slaveLatency issue
                      3.fix 2M PHY extpreamble issue
                      4.fix secondscan & secondinit CSA2 issue
    mesh_lib        : 1,update ltrn replay cache process when replay cache is full
					  2,bugfix SAR Context Allocation fail

### **[misc]**

---

### **Version**: PHY62XX_SDK_3.1.0

### **Change List**

### **[components]**

    driver:
        ppsp        : bugfix ppsp data format bug
        ota_flash   : 1,fix ota hardfault bug(100% progress) 
                      2,update slb service 
        flash       : add spif lock in flash.c
        pwrmgr      : CFG_SRAM_RETENTION_LOW_CURRENT_LDO_ENABLE,default
        i2c         : 1.disable i2c clock when deinit
                      2.enable i2c clock when i2c address update
    mesh:           : 1.set em timer handle to default value
                      2.bugfix get subscription list error
                      3.bugfix stop timer failed when event id can't find
                      4.add debug information for queue full
                      5.rename phyplus model to vendor model

### **[example]**

    ble_mesh        : 1.bugfix mesh switch proxy beacon failed
                      2.add 2M mode

### **[lib]**

    rf.lib          : 1.add load MAC Address from chip Madd function
                      2.update wakeupinit tracking,optimize sleep wakeup procedure
                      3.add ll adv control patch to master
                      4.update rf driver
    mesh_lib        : 1.improve provision ratio
                      2.separate gatt and adv bearear re-transmit time
                      3.set ms_provisioner_addr to unassigned at reset case

### **[misc]**

    bb_rom_sym_m0   : 1.delete more common rom symbol
                      2.add llConnTerminate0 symbol

---
### **Version**: PHY62XX_SDK_3.0.9

### **Change List**

### **[components]**

    driver:
        adc         : optimize adc attenuation mode correction algorithm 
        pwrmgr      : add rf timer irq prio restore
        spi         : optimize spi slvae config
        clock       : optimize WaitMs(use WaitRTCCount)
        bsp_button  : add bsp_button code,it is a key process middleware which use gpio or kscan
        ota_flash   : add crc check when in single no fct mode
        flash       : add load MAC from chipmaddr

    profiles:
        gapbondmgr  : set gapBondMgr_TaskID default value to INVALID_TASK_ID

### **[example]**

    ble_multi       : 1.update multi-role scheduler
                      2.max support 5 link（2 slave and 3 master）
                      3.support DLE , MTU , SMP
    
    ble_mesh        : 1.add ali genie mesh project(include 3KeySwitch,curtain...)
                      2.change scatter_load for ota
                      3.improve provision ratio
                      4.fix proxy timing bug(GAP_DeviceDiscoveryRequest Retry)
                      5.fix mesh light provision failed bug when not use easybonding
                      6.delete sm task
    
    ble_peripheral  : 
        bleUart_AT  : 1.move cliface into source dir, comment out pwrmgr.o file in scatter file 
                      2.simpleBleperipheral:fix scanrsp data bit 
        OTA         : add ota security boot and slb ota

### **[lib]**

    rf.lib          : 1.update rf driver
                      2.add fastadv
                      3.optimize sleep wakeup procedure
                      4.update wakeupinit tracking
    mesh_lib        : 1.fix mesh reset cannot restart bug(provsioner and device)
                      2.unuse friend/lpn function
                      3.unuse MS_ACCESS_PUBLISH_TIMER_SUPPORT function
                      4.change mesh config address(max 4K bytes)
                      5.change nvs init funtion(can choose flash address)
    ble_host        : 1.MTU SIZE marcro define default 247
                      2.ATT/GATT API change uint8 to uint16
    ble_host_multi5 : for ble_multi example

### **[misc]**

    bb_rom_sym_m0   : 1.delete more common rom symbol
                      2.add g_llAdvMode symbol

---

### **Version**: PHY62XX_SDK_3.0.8

### **Change List**

### **[components]**

    driver:
        adc         : update adc_Lambda for adc value calc
        flash       : 1.add cache reset in HAL_CACHE_ENTER/EXIT_BYPASS_SECTION
                      2.add cache bypass in hal_flash_erase_sector/block64/all
        i2c         : fix scl/sda fmux error in i2c_common.c
        kscan       : support low power sleep wakeup
        log         : add LOG_DUMP_BYTE
        pwrmgr      : 1.support multi-io wakeup in standy mode
                      2.turn on CLK_COM , modifed the DEF_CLKG_CONFIG_1
        spi         : fix dma usage issue
        spiflash    : add spiflash bus busy check in spifflash_sector_erase/32KB/64KB
        uart        : fix dma usage issue
    
    ethermind:
        add mesh components
    
    osal:
        osal_snv    : config USE_FS=1 by default, clear NO_FS osal_svn api

    profiles:
        gapbondmgr  : support unlimit GAP_BONDING number by config
                      GAP_BOND_MGR_INDEX_REPLACE

### **[example]**

    ble_central     : add simpleBleCentral demo
    
    ble_mesh        : 1.add aliGenie_mesh_light
                      2.add aliGenie_mesh_multi
                      3.add mesh_gateway
                      4.add mesh_light
                      5.add mesh_switch 
    
    ble_peripheral  : add bleUart_AT 

### **[lib]**

    rf.lib          : 1.update rf driver 
                      2.optimize sleep wakeup time
                      3.add TRNG api(rf_phy_driver.h)
    rf_mst.lib      : for simpleBleCentral demo and ble mesh example

### **[misc]**

    bb_rom_sym_m0   : add more common rom symbol
    jump_table      : add _hard_fault handle for debug

---

### **Version**: PHY62XX_SDK_3.0.7

### **Change List**

    1. pwrmgr:   add hal_pwrmgr_enter_standby api 
    2. dtm: support 125K/500K, ext dtm api
    3. flash: add fix hal_cache_tag_flush bug and add spif_config in hal_cache_init->hw_spif_cache_init
    4. flash: add SPIF polling interval config in hw_spif_cache_config
    5. hiddev: updated for mobile phone compatibility
