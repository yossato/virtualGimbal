<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.2.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="connector">
<packages>
<package name="5035521020">
<smd name="1" x="0.8" y="-1.4" dx="0.24" dy="1" layer="1"/>
<smd name="NC1" x="1.43" y="0.92" dx="0.3" dy="1" layer="1"/>
<wire x1="-1.73" y1="0.835" x2="1.73" y2="0.835" width="0.0762" layer="51"/>
<wire x1="1.73" y1="0.835" x2="1.73" y2="-0.835" width="0.0762" layer="51"/>
<wire x1="1.73" y1="-0.835" x2="-1.73" y2="-0.835" width="0.0762" layer="51"/>
<wire x1="-1.73" y1="-0.835" x2="-1.73" y2="0.835" width="0.0762" layer="51"/>
<rectangle x1="-0.15" y1="0.4" x2="0.15" y2="0.9" layer="51"/>
<rectangle x1="0.25" y1="0.4" x2="0.55" y2="0.9" layer="51"/>
<rectangle x1="0.65" y1="0.4" x2="0.95" y2="0.9" layer="51"/>
<rectangle x1="-0.55" y1="0.4" x2="-0.25" y2="0.9" layer="51"/>
<rectangle x1="-0.95" y1="0.4" x2="-0.65" y2="0.9" layer="51"/>
<rectangle x1="-0.15" y1="-0.9" x2="0.15" y2="-0.4" layer="51"/>
<rectangle x1="0.25" y1="-0.9" x2="0.55" y2="-0.4" layer="51"/>
<rectangle x1="0.65" y1="-0.9" x2="0.95" y2="-0.4" layer="51"/>
<rectangle x1="-0.55" y1="-0.9" x2="-0.25" y2="-0.4" layer="51"/>
<rectangle x1="-0.95" y1="-0.9" x2="-0.65" y2="-0.4" layer="51"/>
<smd name="3" x="0.4" y="-1.4" dx="0.24" dy="1" layer="1"/>
<smd name="5" x="0" y="-1.4" dx="0.24" dy="1" layer="1"/>
<smd name="7" x="-0.4" y="-1.4" dx="0.24" dy="1" layer="1"/>
<smd name="9" x="-0.8" y="-1.4" dx="0.24" dy="1" layer="1"/>
<smd name="2" x="0.8" y="1.4" dx="0.24" dy="1" layer="1"/>
<smd name="4" x="0.4" y="1.4" dx="0.24" dy="1" layer="1"/>
<smd name="6" x="0" y="1.4" dx="0.24" dy="1" layer="1"/>
<smd name="8" x="-0.4" y="1.4" dx="0.24" dy="1" layer="1"/>
<smd name="10" x="-0.8" y="1.4" dx="0.24" dy="1" layer="1"/>
<smd name="NC2" x="-1.43" y="0.92" dx="0.3" dy="1" layer="1"/>
<smd name="NC3" x="1.43" y="-0.92" dx="0.3" dy="1" layer="1"/>
<smd name="NC4" x="-1.43" y="-0.92" dx="0.3" dy="1" layer="1"/>
</package>
<package name="USB">
<smd name="GND" x="0" y="0" dx="1" dy="6.94" layer="1"/>
<smd name="VBUS" x="7" y="0" dx="1" dy="6.94" layer="1"/>
<smd name="D-" x="4.5" y="0.5" dx="1" dy="5.94" layer="1"/>
<smd name="D+" x="2.5" y="0.5" dx="1" dy="5.94" layer="1"/>
</package>
</packages>
<symbols>
<symbol name="CON-2X05">
<wire x1="4.445" y1="6.35" x2="-1.905" y2="6.35" width="0.254" layer="94"/>
<wire x1="-1.905" y1="-6.35" x2="-1.905" y2="6.35" width="0.254" layer="94"/>
<wire x1="-1.905" y1="-6.35" x2="4.445" y2="-6.35" width="0.254" layer="94"/>
<wire x1="4.445" y1="6.35" x2="4.445" y2="-6.35" width="0.254" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-0.635" y2="5.08" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-0.635" y2="2.54" width="0.1524" layer="94"/>
<wire x1="5.08" y1="5.08" x2="3.175" y2="5.08" width="0.1524" layer="94"/>
<wire x1="5.08" y1="2.54" x2="3.175" y2="2.54" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="0" x2="-0.635" y2="0" width="0.1524" layer="94"/>
<wire x1="5.08" y1="0" x2="3.175" y2="0" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="-2.54" x2="-0.635" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="5.08" y1="-2.54" x2="3.175" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="-5.08" x2="-0.635" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="5.08" y1="-5.08" x2="3.175" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="-0.635" y1="5.08" x2="0.635" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-0.635" y1="2.54" x2="0.635" y2="2.54" width="0.4064" layer="94"/>
<wire x1="-0.635" y1="0" x2="0.635" y2="0" width="0.4064" layer="94"/>
<wire x1="-0.635" y1="-2.54" x2="0.635" y2="-2.54" width="0.4064" layer="94"/>
<wire x1="-0.635" y1="-5.08" x2="0.635" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="1.905" y1="5.08" x2="3.175" y2="5.08" width="0.4064" layer="94"/>
<wire x1="1.905" y1="2.54" x2="3.175" y2="2.54" width="0.4064" layer="94"/>
<wire x1="1.905" y1="0" x2="3.175" y2="0" width="0.4064" layer="94"/>
<wire x1="1.905" y1="-2.54" x2="3.175" y2="-2.54" width="0.4064" layer="94"/>
<wire x1="1.905" y1="-5.08" x2="3.175" y2="-5.08" width="0.4064" layer="94"/>
<text x="-1.905" y="7.62" size="1.778" layer="95">&gt;NAME</text>
<text x="-1.905" y="-9.2075" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-5.08" y="5.08" visible="pad" length="short" direction="pas"/>
<pin name="2" x="7.62" y="5.08" visible="pad" length="short" direction="pas" rot="R180"/>
<pin name="3" x="-5.08" y="2.54" visible="pad" length="short" direction="pas"/>
<pin name="4" x="7.62" y="2.54" visible="pad" length="short" direction="pas" rot="R180"/>
<pin name="5" x="-5.08" y="0" visible="pad" length="short" direction="pas"/>
<pin name="6" x="7.62" y="0" visible="pad" length="short" direction="pas" rot="R180"/>
<pin name="7" x="-5.08" y="-2.54" visible="pad" length="short" direction="pas"/>
<pin name="8" x="7.62" y="-2.54" visible="pad" length="short" direction="pas" rot="R180"/>
<pin name="9" x="-5.08" y="-5.08" visible="pad" length="short" direction="pas"/>
<pin name="10" x="7.62" y="-5.08" visible="pad" length="short" direction="pas" rot="R180"/>
</symbol>
<symbol name="USB">
<wire x1="0" y1="5.08" x2="0" y2="-2.54" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="1.27" y2="-3.81" width="0.254" layer="94"/>
<wire x1="1.27" y1="-3.81" x2="3.81" y2="-3.81" width="0.254" layer="94"/>
<wire x1="3.81" y1="-3.81" x2="3.81" y2="6.35" width="0.254" layer="94"/>
<wire x1="3.81" y1="6.35" x2="1.27" y2="6.35" width="0.254" layer="94"/>
<wire x1="1.27" y1="6.35" x2="0" y2="5.08" width="0.254" layer="94"/>
<text x="-2.54" y="8.89" size="1.778" layer="95">&gt;NAME</text>
<text x="10.16" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="1" x="-5.08" y="5.08" visible="pin" direction="in"/>
<pin name="2" x="-5.08" y="2.54" visible="pin" direction="in"/>
<pin name="3" x="-5.08" y="0" visible="pin" direction="in"/>
<pin name="4" x="-5.08" y="-2.54" visible="pin" direction="in"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="5035521020" prefix="CN">
<gates>
<gate name="G$1" symbol="CON-2X05" x="-1.27" y="0"/>
</gates>
<devices>
<device name="" package="5035521020">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="10" pad="10"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
<connect gate="G$1" pin="9" pad="9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="USB" prefix="CN" uservalue="yes">
<gates>
<gate name="G$1" symbol="USB" x="0" y="-1.27"/>
</gates>
<devices>
<device name="" package="USB">
<connects>
<connect gate="G$1" pin="1" pad="VBUS"/>
<connect gate="G$1" pin="2" pad="D-"/>
<connect gate="G$1" pin="3" pad="D+"/>
<connect gate="G$1" pin="4" pad="GND"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="PWR" width="0.254" drill="0">
</class>
<class number="1" name="Signal" width="0.1524" drill="0">
</class>
</classes>
<parts>
<part name="CN4" library="connector" deviceset="USB" device=""/>
<part name="CN3" library="connector" deviceset="5035521020" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="CN4" gate="G$1" x="53.34" y="124.46" rot="MR0">
<attribute name="NAME" x="55.88" y="133.35" size="1.778" layer="95" rot="MR0"/>
<attribute name="VALUE" x="43.18" y="119.38" size="1.778" layer="96" rot="MR90"/>
</instance>
<instance part="CN3" gate="G$1" x="73.66" y="124.46">
<attribute name="NAME" x="71.755" y="132.08" size="1.778" layer="95"/>
<attribute name="VALUE" x="71.755" y="115.2525" size="1.778" layer="96"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="VBUS2" class="0">
<segment>
<pinref part="CN4" gate="G$1" pin="1"/>
<pinref part="CN3" gate="G$1" pin="1"/>
<wire x1="58.42" y1="129.54" x2="68.58" y2="129.54" width="0.1524" layer="91"/>
<label x="58.42" y="129.54" size="1.778" layer="95"/>
</segment>
</net>
<net name="D-2" class="1">
<segment>
<pinref part="CN4" gate="G$1" pin="2"/>
<pinref part="CN3" gate="G$1" pin="3"/>
<wire x1="58.42" y1="127" x2="68.58" y2="127" width="0.1524" layer="91"/>
<label x="58.42" y="127" size="1.778" layer="95"/>
</segment>
</net>
<net name="D+2" class="0">
<segment>
<pinref part="CN4" gate="G$1" pin="3"/>
<pinref part="CN3" gate="G$1" pin="5"/>
<wire x1="58.42" y1="124.46" x2="68.58" y2="124.46" width="0.1524" layer="91"/>
<label x="58.42" y="124.46" size="1.778" layer="95"/>
</segment>
</net>
<net name="GND2" class="0">
<segment>
<pinref part="CN4" gate="G$1" pin="4"/>
<pinref part="CN3" gate="G$1" pin="7"/>
<wire x1="58.42" y1="121.92" x2="68.58" y2="121.92" width="0.1524" layer="91"/>
<label x="58.42" y="121.92" size="1.778" layer="95"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
