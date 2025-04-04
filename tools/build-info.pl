#!/usr/bin/perl
#             __________               __   ___.
#   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
#   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
#   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
#   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
#                     \/            \/     \/    \/            \/
#
# Generate the build-info.release file found on download.rockbox.org

require "./builds.pm";

my $baseurl="https://download.rockbox.org";

print "[release]\n";
print "build_url=$baseurl/release/%VERSION%/rockbox-%TARGET%-%VERSION%.zip\n";
print "voice_url=$baseurl/release/%VERSION%/voice-%TARGET%-%VERSION%-%LANGUAGE%.zip\n";
print "manual_url=$baseurl/release/%VERSION%/rockbox-%TARGET%-%VERSION%%FORMAT%\n";
print "font_url=$baseurl/release/%VERSION%/rockbox-fonts-%VERSION%.zip\n";
print "source_url=$baseurl/release/%VERSION%/rockbox-source-%VERSION%.7z\n";

foreach my $b (&stablebuilds) {
    my $ver;
    if(exists($builds{$b}{release})) {
	$ver = $builds{$b}{release};
    } else {
	$ver = $publicrelease;
    }
    if ($ver <= $publicrelease) {
        print "$b=$ver\n";
    }
}

print "[status]\n";

foreach my $b (&allbuilds) {
    my $ver;
    my $status = $builds{$b}{status};

    if(exists($builds{$b}{release})) {
	$ver = $builds{$b}{release};
    } else {
	$ver = $publicrelease;
    }
    if ($ver > $publicrelease) {
        $status=2;
    }
    print "$b=$status\n";
}

print "\n";
