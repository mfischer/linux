* NI XGE Ethernet controller

Required properties:
- compatible: Should be "ni,xge-enet-2.00"
- reg: Address and length of the register set for the device
- interrupts: Should contain tx and rx interrupt
- interrupt-names: Should be "rx-irq" and "tx-irq"
- phy-mode: See ethernet.txt file in the same directory.
- nvmem-cells: Phandle of nvmem cell containing the mac address
- nvmem-cell-names: Should be "address"

Examples (10G generic PHY):
	nixge0: ethernet@40000000 {
		compatible = "ni,xge-enet-2.00";
		reg = <0x40000000 0x6000>;

		nvmem-cells = <&eth1_addr>;
		nvmem-cell-names = "address";

		interrupts = <0 29 4>, <0 30 4>;
		interrupt-names = "rx-irq", "tx-irq";
		interrupt-parent = <&intc>;

		phy-mode = "xgmii";
		phy-handle = <&ethernet_phy1>;

		ethernet_phy1: ethernet-phy@4 {
			compatible = "ethernet-phy-ieee802.3-c45";
			reg = <4>;
			devices = <0xa>;
		};
	};
