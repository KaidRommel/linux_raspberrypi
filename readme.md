# 完成进度
## probe 函数重写
```c
static int bcm2835_i2c_probe(struct platform_device *pdev)
{
	pr_info("i2c probe!");
	
	// 忽略
	struct bcm2835_i2c_dev *i2c_dev;
	int ret;
	struct i2c_adapter *adap;
	struct clk *mclk;
	u32 bus_clk_rate;
	// 忽略

	// 通过 Arc::try_new()实现
	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;
	// 通过 from_dev实现
	platform_set_drvdata(pdev, i2c_dev);
	// 未实现
	i2c_dev->dev = &pdev->dev;
	// 设置完成标志位，未实现
	init_completion(&i2c_dev->completion);
	
	// 将 I2C 控制器的寄存器地址映射到内核地址空间中，通过 ioremap_resource 实现
	i2c_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(i2c_dev->regs))
		return PTR_ERR(i2c_dev->regs);

	// 获取 i2c 的主时钟，通过clk_get实现
	mclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(mclk))
		return dev_err_probe(&pdev->dev, PTR_ERR(mclk),
				     "Could not get clock\n");
	// 注册和配置 I2C 控制器的总线时钟，通过重写 register 函数实现
	i2c_dev->bus_clk = bcm2835_i2c_register_div(&pdev->dev, mclk, i2c_dev);

	if (IS_ERR(i2c_dev->bus_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(i2c_dev->bus_clk),
				     "Could not register clock\n");

	// 从设备树中读取时钟频率，封装 of_property_read 实现
	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				   &bus_clk_rate);
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "Could not read clock-frequency property\n");
		bus_clk_rate = I2C_MAX_STANDARD_MODE_FREQ;
	}

	// 检查时钟频率设置是否成功，封装 clk_set_rate_exclusive 实现
	ret = clk_set_rate_exclusive(i2c_dev->bus_clk, bus_clk_rate);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret,
				     "Could not set clock frequency\n");

	// 准备并使能时钟，封装 clk_prepare_enable 实现
	ret = clk_prepare_enable(i2c_dev->bus_clk);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't prepare clock");
		goto err_put_exclusive_rate;
	}

	i2c_dev->irq = platform_get_irq(pdev, 0);
	if (i2c_dev->irq < 0) {
		ret = i2c_dev->irq;
		goto err_disable_unprepare_clk;
	}

	ret = request_irq(i2c_dev->irq, bcm2835_i2c_isr, IRQF_SHARED,
			  dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		goto err_disable_unprepare_clk;
	}

	adap = &i2c_dev->adapter;
	i2c_set_adapdata(adap, i2c_dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	snprintf(adap->name, sizeof(adap->name), "bcm2835 (%s)",
		 of_node_full_name(pdev->dev.of_node));
	adap->algo = &bcm2835_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;
	adap->quirks = of_device_get_match_data(&pdev->dev);

	/*
	 * Disable the hardware clock stretching timeout. SMBUS
	 * specifies a limit for how long the device can stretch the
	 * clock, but core I2C doesn't.
	 */
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_CLKT, 0);
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, 0);

	ret = i2c_add_adapter(adap);
	if (ret)
		goto err_free_irq;

	return 0;

err_free_irq:
	free_irq(i2c_dev->irq, i2c_dev);
err_disable_unprepare_clk:
	clk_disable_unprepare(i2c_dev->bus_clk);
err_put_exclusive_rate:
	clk_rate_exclusive_put(i2c_dev->bus_clk);

	return ret;
}
```
