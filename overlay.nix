self: super:
{
  all-seaing-autonomy = super.callPackage ./all_seaing_autonomy/package.nix {};
  all-seaing-bringup = super.callPackage ./all_seaing_bringup/package.nix {};
  all-seaing-controller = super.callPackage ./all_seaing_controller/package.nix {};
  all-seaing-description = super.callPackage ./all_seaing_description/package.nix {};
  all-seaing-driver = super.callPackage ./all_seaing_driver/package.nix {};
  all-seaing-interfaces = super.callPackage ./all_seaing_interfaces/package.nix {};
  all-seaing-navigation = super.callPackage ./all_seaing_navigation/package.nix {};
  all-seaing-perception = super.callPackage ./all_seaing_perception/package.nix {};
  all-seaing-rviz-plugins = super.callPackage ./all_seaing_rviz_plugins/package.nix {};
  all-seaing-utility = super.callPackage ./all_seaing_utility/package.nix {};
}
