{
  description = "host shell for the dual xarm7 ROS 2 humble docker environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs {
        inherit system;
        config.allowUnfree = true;
      };
      # system libraries that UfactoryStudio (an Electron appimage) links
      # against but does not bundle
      appImageLibraries = with pkgs; [
        alsa-lib
        at-spi2-atk
        at-spi2-core
        atk
        cairo
        cups
        dbus
        expat
        gdk-pixbuf
        glib
        gtk3
        libdrm
        libgbm
        libx11
        libxcb
        libxcomposite
        libxdamage
        libxext
        libxfixes
        libxkbcommon
        libxrandr
        libxshmfence
        nspr
        nss
        pango
      ];
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        packages = with pkgs; [
          docker
          git
          xhost
        ];
        shellHook = ''
          # nix curl/git need the host CA bundle path spelled out
          export SSL_CERT_FILE=/etc/ssl/certs/ca-bundle.crt
          export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath appImageLibraries}:''${LD_LIBRARY_PATH:-}"
        '';
      };
    };
}
